import rclpy
from rclpy.node import Node
import signal
import subprocess
import pathlib


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.declare_parameter('record_path', './NavTrialBag')
        self.declare_parameter('topics', [])

        target = self.get_parameter('record_path').value
        full_target = pathlib.Path(target).resolve()
        self.cmd = ['ros2', 'bag', 'record']
        self.cmd += ['-o', target]
        self.cmd += ['-s', 'mcap']
        topics = ['/tf', '/tf_static', '/clock', '/trial_goal_pose', '/trial_goal_pose_2d', '/navigation_result']
        topics += self.get_parameter('topics').value
        self.cmd += topics
        self.get_logger().info(f'Recording {len(topics)} topics to {full_target}')

        self.process = subprocess.Popen(self.cmd)

    def __del__(self):
        self.get_logger().info('Bag Conclusion')
        self.process.send_signal(signal.SIGINT)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = BagRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
