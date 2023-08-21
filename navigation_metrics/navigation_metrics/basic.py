from action_msgs.msg import GoalStatus

from .metric import RecordedMessage, nav_metric, metric_conversion_function


@metric_conversion_function('/trial_goal_pose')
def convert_to_trial(data):
    goal_pose_msgs = data['/goal_pose']
    if goal_pose_msgs:
        start_rmsg = goal_pose_msgs[0]
        seq = [RecordedMessage(start_rmsg.t, start_rmsg.msg)]
        print(seq)
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
