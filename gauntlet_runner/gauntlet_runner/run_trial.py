import rclpy
from simple_actions.simple_client import SimpleActionClient, ResultCode
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class TrialRunner(Node):
    def __init__(self):
        super().__init__('trial_runner')
        self.nav_action_client = SimpleActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.completed = None
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 2.0
        self.nav_action_client.send_goal(goal_msg, self.done)

    def done(self, result_code, result):
        if result_code == ResultCode.REJECTED:
            self.create_timer(1.0, self.send_goal)
            return

        self.get_logger().info(str(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrialRunner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
