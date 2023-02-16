import rclpy
from simple_actions.simple_client import SimpleActionClient, ResultCode
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class TrialRunner(Node):
    def __init__(self):
        super().__init__('trial_runner')
        self.declare_parameter('goal_pose_x', 0.0)
        self.declare_parameter('goal_pose_y', 0.0)
        self.declare_parameter('goal_pose_yaw', 0.0)

        self.nav_action_client = SimpleActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.completed = None
        self.timer = None
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.get_parameter('goal_pose_x').value
        goal_msg.pose.pose.position.y = self.get_parameter('goal_pose_y').value

        yaw = self.get_parameter('goal_pose_yaw').value
        quat = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.w = quat[0]
        goal_msg.pose.pose.orientation.x = quat[1]
        goal_msg.pose.pose.orientation.y = quat[2]
        goal_msg.pose.pose.orientation.z = quat[3]
        self.get_logger().info(f'Sending goal '
                               f'{goal_msg.pose.pose.position.x:.2f} '
                               f'{goal_msg.pose.pose.position.y:.2f} '
                               f'{yaw:.2f}')
        self.nav_action_client.send_goal(goal_msg, self.done)

        if self.timer:
            self.timer.cancel()

    def done(self, result_code, result):
        if result_code == ResultCode.REJECTED:
            self.timer = self.create_timer(1.0, self.send_goal)
            return

        self.get_logger().info(str(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrialRunner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
