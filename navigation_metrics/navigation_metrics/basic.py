from action_msgs.msg import GoalStatus

from .metric import nav_metric


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
