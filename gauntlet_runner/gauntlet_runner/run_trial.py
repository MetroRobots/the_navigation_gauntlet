import rclpy
from simple_actions.simple_client import SimpleActionClient, ResultCode
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_2d_msgs.msg import Pose2DStamped


class TrialRunner(Node):
    def __init__(self):
        super().__init__('trial_runner')
        self.declare_parameter('goal_pose_x', 0.0)
        self.declare_parameter('goal_pose_y', 0.0)
        self.declare_parameter('goal_pose_yaw', 0.0)

        self.nav_action_client = SimpleActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.completed = None
        self.logger = self.get_logger()
        self.timer = None

        self.pub_3d = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.pub_2d = self.create_publisher(Pose2DStamped, '/goal_pose_2d', 1)

        # Construct Goal
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose.header.frame_id = 'map'
        self.goal_msg.pose.pose.position.x = self.get_parameter('goal_pose_x').value
        self.goal_msg.pose.pose.position.y = self.get_parameter('goal_pose_y').value

        yaw = self.get_parameter('goal_pose_yaw').value
        quat = quaternion_from_euler(0, 0, yaw)
        self.goal_msg.pose.pose.orientation.w = quat[0]
        self.goal_msg.pose.pose.orientation.x = quat[1]
        self.goal_msg.pose.pose.orientation.y = quat[2]
        self.goal_msg.pose.pose.orientation.z = quat[3]

        self.pose2d = Pose2DStamped()
        self.pose2d.header = self.goal_msg.pose.header
        self.pose2d.pose.x = self.goal_msg.pose.pose.position.x
        self.pose2d.pose.y = self.goal_msg.pose.pose.position.y
        self.pose2d.pose.theta = yaw

        self.logger.info(f'Sending goal '
                         f'{self.pose2d.pose.x:.2f} '
                         f'{self.pose2d.pose.y:.2f} '
                         f'{self.pose2d.pose.theta:.2f}')
        self.pub_3d.publish(self.goal_msg.pose)
        self.pub_2d.publish(self.pose2d)

        self.send_goal()

    def send_goal(self):
        self.nav_action_client.send_goal(self.goal_msg, self.done)

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
