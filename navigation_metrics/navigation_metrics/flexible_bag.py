from rosbag2_py import SequentialReader, SequentialWriter
from rosbag2_py import StorageOptions, StorageFilter, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

import collections
import pathlib
import shutil
import tempfile

# Bag Message type for pairing a message with the floating point time it was recorded in the bag file
BagMessage = collections.namedtuple('BagMessage', 't msg')


def get_message_name(obj):
    """ This is a hacky way to get 'std_msgs/msg/String' from a String object
        that is only a little surprising isn't built into the API """
    c = obj.__class__
    pkg_name, interface_type, _ = c.__module__.split('.')
    return '/'.join([pkg_name, interface_type, c.__qualname__])


class MissingTopicException(RuntimeError):
    def __init__(self, topic):
        RuntimeError.__init__(self, f'Cannot find topic: {topic}')


class FlexibleBag:
    """
    A flexible bag container that allows for a variety of data access modes

    bag[topic_string] - Results in a list of BagMessage objects for the topic
    bag[topic1, topic2] - Results in a list of BagMessage tuples, where timestamps are as close as possible

    You can also add new data with
    bag[topic] = list of BagMessages
    Added data will be stored to disk on exit if write_mods=True
    """

    # We maintain a static dictionary of conversion functions, where the key is a topic name
    # and the value is a function returning a list of BagMessages to associate with that topic
    conversion_functions = {}

    def __init__(self, path, storage_id='sqlite3', serialization_format='cdr', write_mods=True, initial_cache=False):
        self.path = path
        self.storage_options = StorageOptions(str(self.path), storage_id)
        self.converter_options = ConverterOptions(serialization_format, serialization_format)
        self.write_mods = write_mods
        self.cached_topics = {}
        self.new_topics = {}

        # Read Initial Topic Information
        reader = SequentialReader()
        reader.open(self.storage_options, self.converter_options)
        self.topic_types = reader.get_all_topics_and_types()
        self.type_map = {}
        self.topics_by_type = collections.defaultdict(set)
        for tmeta in self.topic_types:
            self.type_map[tmeta.name] = tmeta.type
            self.topics_by_type[tmeta.type].add(tmeta.name)

        # Cache all the data
        if initial_cache:
            self.read_remaining_topics()

    def __del__(self):
        """ Destructor to write data to file """
        if not self.new_topics or not self.write_mods:
            return

        with tempfile.TemporaryDirectory() as tmpdirname:
            # Save to temp dir
            temp_bag_file = pathlib.Path(tmpdirname) / self.path.stem
            self.save(temp_bag_file)

            # Remove original
            shutil.rmtree(self.path)

            # Replace with new file
            shutil.move(str(temp_bag_file), str(self.path))

    def __contains__(self, topic):
        """Returns true if a given topic is available from the bag file"""
        return topic in self.type_map

    def __getitem__(self, arg):
        """
        If the arg is a string, results in a list of BagMessage objects where topic == arg
        """
        if isinstance(arg, str):
            return self.get_single_topic(arg)
        else:
            return self.read_multiple_topics(arg)

    def __setitem__(self, topic, sequence):
        if topic in self.type_map:
            raise RuntimeError('Topic already contained in bag')
        elif not sequence:
            raise RuntimeError('Sequence empty and cannot determine msg type')
        self.new_topics[topic] = sequence

        msgtype = get_message_name(sequence[0].msg)
        tmeta = TopicMetadata(name=topic, type=msgtype,
                              serialization_format=self.converter_options.output_serialization_format)

        self.topic_types.append(tmeta)
        self.type_map[tmeta.name] = tmeta.type
        self.topics_by_type[tmeta.type].add(tmeta.name)

    def get_single_topic(self, topic):
        """
        Returns a list of BagMessage objects with the given topic
        """
        if topic in self.new_topics:
            # New topic
            return self.new_topics[topic]
        elif topic in self.type_map:
            # Existing topic
            if topic not in self.cached_topics:
                self._cache_topics([topic])
            return self.cached_topics[topic]
        elif topic in FlexibleBag.conversion_functions:
            # Topic to be converted
            fne = FlexibleBag.conversion_functions[topic]
            result = fne(self)
            if result is None:
                raise MissingTopicException(topic)
            self[topic] = result
            return result
        else:
            raise MissingTopicException(topic)

    def read_multiple_topics(self, topics, allow_reuse=False):
        """
        Uses a greedy algorithm to match up messages from multiple topics to their closest neighbors

        Does NOT necessarily return all data from the topics.
        """
        seqs = {}
        shortest = None
        shortest_length = None

        # Save the sequences and determine which has the fewest messages
        for topic in topics:
            seqs[topic] = self.get_single_topic(topic)
            n = len(seqs[topic])
            if shortest is None or shortest_length > n:
                shortest = topic
                shortest_length = n

        # Create a dictionary for each of the messages in the shortest sequence
        frames = [{shortest: bmsg} for bmsg in seqs[shortest]]

        # Go through the other topics and find the closest messages
        for topic, seq in seqs.items():
            if topic == shortest:
                continue

            # All options
            options = []
            for frame_i, frame in enumerate(frames):
                for seq_j, bmsg in enumerate(seq):
                    dt = abs(frame[shortest].t - bmsg.t)
                    options.append((dt, frame_i, seq_j))

            seen_js = set()
            matches = 0
            for dt, frame_i, seq_j in sorted(options):
                frame = frames[frame_i]
                if topic in frame:
                    continue
                if allow_reuse or seq_j not in seen_js:
                    frame[topic] = seq[seq_j]
                    seen_js.add(seq_j)
                    matches += 1
                    if matches == len(frames):
                        break

        # Convert dictionaries to tuples and yield
        for frame in frames:
            yield tuple(frame[topic] for topic in topics)

    def read_remaining_topics(self):
        topics = set(self.type_map.keys())
        for topic in self.cached_topics:
            if topic in topics:
                topics.remove(topic)
        self._cache_topics(list(topics))

    def get_topics_by_type(self, msg_type_s):
        """Returns a list of topics whose type matches the string passed in"""
        return sorted(self.topics_by_type[msg_type_s])

    def save(self, output_path):
        """Save results to file"""
        self.read_remaining_topics()

        writer = SequentialWriter()
        writer.open(StorageOptions(str(output_path), self.storage_options.storage_id), self.converter_options)
        for tmeta in self.topic_types:
            writer.create_topic(tmeta)
            for ts, msg in self.get_single_topic(tmeta.name):
                writer.write(tmeta.name, serialize_message(msg), int(ts * 1e9))

    def _cache_topics(self, topics):
        """
        Cache all of the topics into memory
        """
        for topic in topics:
            self.cached_topics[topic] = []

        reader = SequentialReader()
        reader.open(self.storage_options, self.converter_options)
        storage_filter = StorageFilter(topics=topics)
        reader.set_filter(storage_filter)

        while reader.has_next():
            (topic, rawdata, timestamp) = reader.read_next()
            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(rawdata, msg_type)
            self.cached_topics[topic].append(BagMessage(timestamp / 1e9, msg))

    def __repr__(self):
        return f'FlexibleBag({self.path.stem})'


# Decorator function for adding conversion functions
def flexible_bag_converter_function(topic):
    def actual_decorator(f):
        FlexibleBag.conversion_functions[topic] = f
        return f
    return actual_decorator
