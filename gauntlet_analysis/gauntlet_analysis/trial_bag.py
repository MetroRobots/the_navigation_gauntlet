from rosbag2_py import SequentialReader, SequentialWriter
from rosbag2_py import StorageOptions, ConverterOptions, StorageFilter, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from navigation_metrics import RecordedMessage, get_conversion_functions
import pathlib
import tempfile
import shutil


def get_message_name(obj):
    c = obj.__class__
    pkg_name, interface_type, _ = c.__module__.split('.')
    return '/'.join([pkg_name, interface_type, c.__qualname__])


class TrialBag:
    def __init__(self, path, parameters={}, write_mods=False, read_everything=False, serialization_format='cdr'):
        self.bag_reader = None
        self.path = path
        self.parameters = parameters
        self.write_mods = write_mods
        self.serialization_format = serialization_format
        self.cached_topics = {}
        self.new_topics = {}

        self.bag_options = (StorageOptions(str(self.path), 'sqlite3'),
                            ConverterOptions(serialization_format, serialization_format))
        reader = SequentialReader()
        reader.open(*self.bag_options)
        self.topic_types = reader.get_all_topics_and_types()
        self.type_map = {tmeta.name: tmeta.type for tmeta in self.topic_types}

        if write_mods or read_everything:
            self.read_topics(list(self.type_map.keys()))

    def __del__(self):
        if not self.new_topics or not self.write_mods:
            return

        with tempfile.TemporaryDirectory() as tmpdirname:
            temp_bag_file = pathlib.Path(tmpdirname) / self.path.stem

            self.save(temp_bag_file)

            shutil.rmtree(self.path)

            shutil.move(str(temp_bag_file), str(self.path))

    def __getitem__(self, arg):
        if isinstance(arg, str):
            return self.get_single_topic(arg)
        else:
            return self.read_multiple_topics(arg)

    def __contains__(self, topic):
        return topic in self.type_map or topic in get_conversion_functions()

    def get_single_topic(self, topic):
        if topic in self.type_map:
            # Existing topic
            if topic not in self.cached_topics:
                self.read_topics([topic])
            return self.cached_topics[topic]
        elif topic in get_conversion_functions():
            # New Topic
            fne = get_conversion_functions()[topic]
            if topic not in self.new_topics:
                self.new_topics[topic] = fne(self)
            return self.new_topics[topic]
        else:
            raise RuntimeError(f'Cannot find {topic} in bag or conversion functions')

    def read_topics(self, topics):
        for topic in topics:
            self.cached_topics[topic] = []

        reader = SequentialReader()
        reader.open(*self.bag_options)
        storage_filter = StorageFilter(topics=topics)
        reader.set_filter(storage_filter)

        while reader.has_next():
            (topic, rawdata, timestamp) = reader.read_next()
            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(rawdata, msg_type)
            self.cached_topics[topic].append(RecordedMessage(timestamp / 1e9, msg))

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
        writer = SequentialWriter()
        writer.open(StorageOptions(str(output_path), 'sqlite3'),
                    ConverterOptions(self.serialization_format, self.serialization_format))

        for tmeta in self.topic_types:
            writer.create_topic(tmeta)

            for ts, msg in self.cached_topics[tmeta.name]:
                writer.write(tmeta.name, serialize_message(msg), int(ts * 1e9))

            for topic, msgs in self.new_topics.items():
                msgtype = get_message_name(msgs[0][1])
                tmeta = TopicMetadata(name=topic, type=msgtype,
                                      serialization_format=self.serialization_format)

                writer.create_topic(tmeta)

                for ts, msg in msgs:
                    writer.write(topic, serialize_message(msg), int(ts * 1e9))

    def __repr__(self):
        return f'TrialBag({self.path.stem})'
