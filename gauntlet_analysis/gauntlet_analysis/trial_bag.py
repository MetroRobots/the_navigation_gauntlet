from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.typesys.types import FIELDDEFS
from rosidl_runtime_py import get_interface_path
from navigation_metrics import RecordedMessage, get_conversion_functions
import pathlib
import tempfile
import shutil
import re


def is_registered_type(msgtype):
    return msgtype in FIELDDEFS


def get_subtypes(tree):
    if isinstance(tree, tuple):
        if isinstance(tree[0], str):
            field_type = tree[1]
            if isinstance(field_type, tuple):
                if field_type[0].value == 2:
                    yield field_type[1]
                elif field_type[0].value == 4:
                    base_type = field_type[1][0]
                    yield base_type[1]
            return

    for k in tree:
        yield from get_subtypes(k)


def register_new_type(type_name):
    queue = [type_name]
    types_to_register = {}

    while queue:
        msgtype = queue.pop(0)
        i_path = pathlib.Path(get_interface_path(msgtype))
        msg_text = i_path.read_text()
        type_dict = get_types_from_msg(msg_text, msgtype)
        types_to_register.update(type_dict)
        for subtype in get_subtypes(type_dict[msgtype]):
            if subtype in types_to_register or is_registered_type(subtype):
                continue
            queue.append(subtype)

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
        if not is_registered_type(msgtype):
            register_new_type(msgtype)
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
    def __init__(self, path, parameters={}, write_mods=False):
        self.bag_reader = None
        self.path = path
        self.parameters = parameters
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

    def get_param(self, name, default_value=None):
        return self.parameters.get(name, default_value)

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
