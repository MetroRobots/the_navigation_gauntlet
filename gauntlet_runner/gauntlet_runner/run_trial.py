import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_2d_msgs.msg import Pose2DStamped
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock


class TrialRunner(Node):
    def __init__(self):
        super().__init__('trial_runner')
        self.declare_parameter('send_goal', True)
        self.declare_parameter('goal_pose_x', 0.0)
        self.declare_parameter('goal_pose_y', 0.0)
        self.declare_parameter('goal_pose_yaw', 0.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_delay', 0.0)
        self.declare_parameter('timeout', 600.0)
        self.completed = None
        self.logger = self.get_logger()
        self.timer = None
        self.timeout_timer = None
        self.goal_timer = None

        latching_qos = QoSProfile(depth=1,
                                  durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.got_clock = False
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 1)
        while not self.got_clock:
            self.logger.info('waiting for clock')
            rclpy.spin_once(self)

        self.pub_status = self.create_publisher(GoalStatus, '/navigation_result', 1)

        self.should_send_goal = self.get_parameter('send_goal').value

        if self.should_send_goal:
            self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            printed = False
            while not self.nav_action_client.wait_for_server(timeout_sec=5.0):
                if not printed:
                    self.logger.warn('Waiting for /navigate_to_pose')
                    printed = True
            if printed:
                self.logger.info('Connected to /navigate_to_pose')

            self.pub_3d = self.create_publisher(PoseStamped, '/trial_goal_pose', latching_qos)
            self.pub_2d = self.create_publisher(Pose2DStamped, '/trial_goal_pose_2d', latching_qos)

            # Construct Goal
            self.goal_msg = NavigateToPose.Goal()
            self.goal_msg.pose.header.frame_id = self.get_parameter('goal_frame').value
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

            goal_delay = self.get_parameter('goal_delay').value
            if goal_delay == 0.0:
                self.send_goal()
            else:
                self.goal_timer = self.create_timer(goal_delay, self.send_goal)

        self.timeout_length = self.get_parameter('timeout').value
        if self.timeout_length > 0.0:
            self.timeout_timer = self.create_timer(self.timeout_length, self._timeout_cb)

    def clock_cb(self, msg):
        self.got_clock = True
        self.clock_sub.destroy()
        self.clock_sub = None

    def send_goal(self):
        self._goal_future = self.nav_action_client.send_goal_async(self.goal_msg)
        self._goal_future.add_done_callback(self._goal_response_callback)

        if self.timer:
            self.timer.cancel()
        if self.goal_timer:
            self.goal_timer.cancel()

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.timer = self.create_timer(1.0, self.send_goal)
            return

        self.logger.info(f'Sending goal '
                         f'{self.pose2d.pose.x:.2f} '
                         f'{self.pose2d.pose.y:.2f} '
                         f'{self.pose2d.pose.theta:.2f}')
        self.pub_3d.publish(self.goal_msg.pose)
        self.pub_2d.publish(self.pose2d)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()

        status_msg = GoalStatus()
        status_msg.status = result.status
        self.pub_status.publish(status_msg)

        if self.timeout_timer:
            self.timeout_timer.cancel()

        self.logger.info('Navigation trial complete!')
        rclpy.shutdown()

    def _timeout_cb(self):
        status_msg = GoalStatus()
        status_msg.status = GoalStatus.STATUS_ABORTED
        self.pub_status.publish(status_msg)

        # TODO: Cancel action server?
        # TODO: Rely on other callback
        self.logger.info(f'Navigation trial timed out after {self.timeout_length} seconds.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrialRunner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
