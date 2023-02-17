from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from rosidl_runtime_py import get_interface_path
import pathlib

types_to_register = {}
for type_name in ['action_msgs/msg/GoalStatus',
                  'action_msgs/msg/GoalInfo']:
    i_path = pathlib.Path(get_interface_path(type_name))
    msg_text = i_path.read_text()
    types_to_register.update(get_types_from_msg(msg_text, type_name))
register_types(types_to_register)


class TrialBag:
    def __init__(self, path):
        self.bag_reader = None
        self.path = path
        self.cached_topics = {}

        self.bag_reader = Reader(str(path))
        self.bag_reader.open()

    def __del__(self):
        if self.bag_reader:
            self.bag_reader.close()

    def get_topic(self, topic):
        if topic in self.cached_topics:
            return self.cached_topics[topic]

        self.cached_topics[topic] = []

        connections = [conn for conn in self.bag_reader.connections.values() if conn.topic == topic]

        for connection, timestamp, rawdata in self.bag_reader.messages(connections=connections):
            ts = timestamp / 1e9
            self.cached_topics[topic].append((ts, deserialize_cdr(rawdata, connection.msgtype)))
        return self.cached_topics[topic]

    def __repr__(self):
        return f'TrialBag({self.path.stem})'
