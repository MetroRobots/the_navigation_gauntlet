from action_msgs.msg import GoalStatus

from navigation_metrics.metric import nav_metric
from navigation_metrics import BagMessage, flexible_bag_converter_function


@flexible_bag_converter_function('/trial_goal_pose')
def convert_to_trial(data):
    goal_pose_msgs = data['/goal_pose']
    if goal_pose_msgs:
        start_bmsg = goal_pose_msgs[0]
        seq = [BagMessage(start_bmsg.t, start_bmsg.msg)]
        return seq


@nav_metric
def time_to_start(data):
    """
    Number of seconds from when the goal pose was sent to the first moving command.

    Units: seconds
    Topics: /trial_goal_pose, /cmd_vel
    """
    goals = data['/trial_goal_pose']
    goal_t = goals[0].t
    cmds = data['/cmd_vel']
    for t, cmd in cmds:
        if t < goal_t:
            continue
        if cmd.linear.x != 0.0 and cmd.angular.z != 0.0:
            return t - goal_t


@nav_metric
def completed(data):
    """
    Whether the navigation action successfully completed.

    Units: Boolean
    Topics: /navigation_result
    """
    results = data['/navigation_result']
    if not results:
        return False
    return results[0].msg.status == GoalStatus.STATUS_SUCCEEDED


@nav_metric
def total_time(data):
    """
    The total time of the trial

    Units: seconds
    """
    return data.length()
