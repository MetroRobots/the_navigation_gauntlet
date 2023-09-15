from action_msgs.msg import GoalStatus

from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import BagMessage, flexible_bag_converter_function
from navigation_metrics.util import point_distance


@flexible_bag_converter_function('/trial_goal_pose')
def convert_to_trial(data):
    goal_pose_msgs = data['/goal_pose']
    if goal_pose_msgs:
        start_bmsg = goal_pose_msgs[0]
        seq = [BagMessage(start_bmsg.t, start_bmsg.msg)]
        return seq


@flexible_bag_converter_function('/navigation_result')
def find_endpoint(data):
    goal_pose_msgs = data['/trial_goal_pose']
    cmds = data['/cmd_vel']
    if not goal_pose_msgs or not cmds:
        return
    goal = goal_pose_msgs[0].msg

    # Find last active cmd_vel
    i = len(cmds) - 1
    while i >= 0:
        t, msg = cmds[i]
        if abs(msg.linear.x) < 1e-2 and abs(msg.angular.z) < 1e-2:
            i -= 1
        else:
            break

    last_cmd_t = cmds[i + 1].t

    global_frame = data.get_parameter('global_frame', 'map')
    robot_frame = data.get_parameter('robot_frame', 'base_link')
    distance_tolerance = data.get_parameter('distance_tolerance', 0.5)

    for t, msg in data.get_messages_by_time('/tf', last_cmd_t):
        for transform in msg.transforms:
            if transform.header.frame_id == global_frame and transform.child_frame_id == robot_frame:
                d = point_distance(goal.pose.position, transform.transform.translation)

                new_msg = GoalStatus()
                if d < distance_tolerance:
                    new_msg.status = GoalStatus.STATUS_SUCCEEDED
                else:
                    new_msg.status = GoalStatus.STATUS_ABORTED

                seq = [BagMessage(t, new_msg)]
                return seq


@nav_metric
def time_to_start(data):
    goals = data['/trial_goal_pose']
    goal_t = goals[0].t
    cmds = data['/cmd_vel']
    for t, cmd in cmds:
        if cmd.linear.x != 0.0 and cmd.angular.z != 0.0:
            return t - goal_t


@nav_metric
def completed(data):
    results = data['/navigation_result']
    if not results:
        return False
    return results[0].msg.status == GoalStatus.STATUS_SUCCEEDED


@nav_metric
def total_time(data):
    goals = data['/trial_goal_pose']
    results = data['/navigation_result']

    if not goals or not results:
        return

    return results[0].t - goals[0].t
