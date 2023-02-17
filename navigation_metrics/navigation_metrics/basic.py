from .metric import nav_metric
from action_msgs.msg import GoalStatus


@nav_metric
def completed(data):
    results = data.get_topic('/navigation_result')
    if not results:
        return False
    _, result = results[0]
    return result.status == GoalStatus.STATUS_SUCCEEDED
