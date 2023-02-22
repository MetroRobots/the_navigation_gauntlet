from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from rosidl_runtime_py import get_interface_path
from navigation_metrics import RecordedMessage, get_conversion_functions
import pathlib
import tempfile
import shutil
import re

types_to_register = {}
for type_name in ['action_msgs/msg/GoalStatus',
                  'action_msgs/msg/GoalInfo',
                  'nav_2d_msgs/msg/Pose2DStamped',
                  'nav_2d_msgs/msg/Twist2DStamped',
                  'nav_2d_msgs/msg/Twist2D']:
    i_path = pathlib.Path(get_interface_path(type_name))
    msg_text = i_path.read_text()
    types_to_register.update(get_types_from_msg(msg_text, type_name))
register_types(types_to_register)


def get_message_name(obj):
    c = obj.__class__
    pkg_name, interface_type, _ = c.__module__.split('.')
    return '/'.join([pkg_name, interface_type, c.__qualname__])


class CustomDeserializer:
    SEQUENCE_PATTERN = re.compile(r'sequence<([_\w]+)/([_\w]+)>')

    def __init__(self):
        self.msg_types = {}

    def __call__(self, rawdata, msgtype):
        basic = deserialize_cdr(rawdata, msgtype)

        pkg, msg_name = msgtype.split('/msg/')
        actual_msg = self.get_msg_class(pkg, msg_name)()
        self.copy_fields(actual_msg, basic)
        return actual_msg

    def get_msg_class(self, pkg, msg_name):
        key = pkg, msg_name
        if key not in self.msg_types:
            mod = __import__(f'{pkg}.msg').msg
            self.msg_types[key] = getattr(mod, msg_name)
        return self.msg_types[key]

    def copy_fields(self, target, src):
        for field, field_type in target.get_fields_and_field_types().items():
            m = CustomDeserializer.SEQUENCE_PATTERN.match(field_type)
            if m:
                cls = self.get_msg_class(*m.groups())
                dest = getattr(target, field)
                for array_mem in getattr(src, field):
                    array_msg = cls()
                    self.copy_fields(array_msg, array_mem)
                    dest.append(array_msg)
            elif '/' in field_type:
                self.copy_fields(getattr(target, field), getattr(src, field))
            else:
                setattr(target, field, getattr(src, field))


class TrialBag:
    def __init__(self, path, write_mods=False):
        self.bag_reader = None
        self.path = path
        self.write_mods = write_mods
        self.cached_topics = {}
        self.new_topics = {}
        self.deserializer = CustomDeserializer()

        self.bag_reader = Reader(str(path))
        self.bag_reader.open()

        self.connection_map = {conn.topic: conn for conn in self.bag_reader.connections.values()}

    def __del__(self):
        if not self.bag_reader:
            return

        if not self.new_topics or not self.write_mods:
            # Just close the reader
            self.bag_reader.close()
            return

        with tempfile.TemporaryDirectory() as tmpdirname:
            temp_bag_file = pathlib.Path(tmpdirname) / self.path.stem

            self.save(temp_bag_file)
            self.bag_reader.close()

            shutil.rmtree(self.path)

            shutil.move(str(temp_bag_file), str(self.path))

    def __getitem__(self, arg):
        if isinstance(arg, str):
            return self.get_single_topic(arg)
        else:
            return self.read_multiple_topics(arg)

    def get_single_topic(self, topic):
        if topic in self.connection_map:
            # Existing topic
            if topic not in self.cached_topics:
                self.cached_topics[topic] = self.read_topic_sequence(topic)
            return self.cached_topics[topic]
        elif topic in get_conversion_functions():
            # New Topic
            fne = get_conversion_functions()[topic]
            if topic not in self.new_topics:
                self.new_topics[topic] = fne(self)
            return self.new_topics[topic]
        else:
            raise RuntimeError(f'Cannot find {topic} in bag or conversion functions')

    def read_topic_sequence(self, topic):
        seq = []
        for conn, timestamp, rawdata in self.bag_reader.messages(connections=[self.connection_map[topic]]):
            ts = timestamp / 1e9
            seq.append(RecordedMessage(ts, self.deserializer(rawdata, conn.msgtype)))
        return seq

    def read_multiple_topics(self, topics):
        seqs = {}
        shortest = None
        shortest_length = None

        for topic in topics:
            seq = self.get_single_topic(topic)
            seqs[topic] = seq
            n = len(seq)
            if shortest is None or shortest_length > n:
                shortest = topic
                shortest_length = n

        frames = []
        for rmsg in seqs[shortest]:
            frame = {}
            frame[shortest] = rmsg
            frames.append(frame)

        for topic, seq in seqs.items():
            if topic == shortest:
                continue

            for frame in frames:
                for rmsg in seq:
                    if topic not in frame:
                        frame[topic] = rmsg
                        continue
                    existing_dt = abs(frame[shortest].t - frame[topic].t)
                    new_dt = abs(frame[shortest].t - rmsg.t)
                    if new_dt < existing_dt:
                        frame[topic] = rmsg

        for frame in frames:
            rmsgs = []
            for topic in topics:
                rmsgs.append(frame[topic])
            yield rmsgs

    def save(self, output_path):
        with Writer(output_path) as writer:
            out_connections = {}
            for conn_id, conn in self.bag_reader.connections.items():
                out_connections[conn.topic] = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    conn.serialization_format,
                    conn.offered_qos_profiles,
                )

            for conn, timestamp, data in self.bag_reader.messages():
                writer.write(out_connections[conn.topic], timestamp, data)

            for topic, msgs in self.new_topics.items():
                msgtype = get_message_name(msgs[0][1])
                conn = writer.add_connection(topic, msgtype)
                for ts, msg in msgs:
                    timestamp = int(ts * 1e9)
                    data = serialize_cdr(msg, conn.msgtype)
                    writer.write(conn, timestamp, data)

    def __repr__(self):
        return f'TrialBag({self.path.stem})'
