from action_msgs.msg import GoalStatus

from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import BagMessage, flexible_bag_converter_function


@flexible_bag_converter_function('/trial_goal_pose')
def convert_to_trial(data):
    goal_pose_msgs = data['/goal_pose']
    if goal_pose_msgs:
        start_bmsg = goal_pose_msgs[0]
        seq = [BagMessage(start_bmsg.t, start_bmsg.msg)]
        return seq


@flexible_bag_converter_function('/navigation_result')
def find_endpoint(data):
    goal_pose_msgs = data['/goal_pose']
    if goal_pose_msgs:
        start_bmsg = goal_pose_msgs[0]
        seq = [BagMessage(start_bmsg.t, start_bmsg.msg)]
        return seq


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
