from action_msgs.msg import GoalStatus

from .metric import nav_metric


@nav_metric
def completed(data):
    results = data['/navigation_result']
    if not results:
        return False
    _, result = results[0]
    return result.status == GoalStatus.STATUS_SUCCEEDED


@nav_metric
def total_time(data):
    goals = data['/trial_goal_pose']
    results = data['/navigation_result']

    if not goals or not results:
        return

    t0 = goals[0][0]
    t1 = results[0][0]

    return t1 - t0
