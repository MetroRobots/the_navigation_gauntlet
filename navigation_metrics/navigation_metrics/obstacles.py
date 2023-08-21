from std_msgs.msg import Float32

from polygon_utils.shapely_lib import polygon_from_msg, point_from_msg

from .metric import RecordedMessage, nav_metric, metric_conversion_function
from .util import metric_min, metric_max, average


@metric_conversion_function('/obstacle_clearance')
def calculate_obstacle_clearance(data):
    polygons_msg = data['/polygon_map'][0].msg
    spolygons = [polygon_from_msg(poly) for poly in polygons_msg.polygons]

    seq = []
    for t, msg in data['/path']:
        spoint = point_from_msg(msg.pose.position)
        ds = [spoly.exterior.distance(spoint) for spoly in spolygons]
        fmsg = Float32()
        fmsg.data = min(ds)
        seq.append(RecordedMessage(t, fmsg))
    return seq


@nav_metric
def minimum_clearing_distance(data):
    return metric_min(data['/obstacle_clearance'])


@nav_metric
def average_clearing_distance(data):
    return average(data['/obstacle_clearance'])


@nav_metric
def maximum_clearing_distance(data):
    return metric_max(data['/obstacle_clearance'])


@nav_metric
def total_collisions(data):
    start_t = data['/trial_goal_pose'][0].t
    end_t = data['/navigation_result'][0].t

    collision_topics = data.get_topics_by_type('collision_msgs/msg/Collisions')
    total = 0
    for topic in collision_topics:
        for t, msg in data[topic]:
            if t < start_t:
                continue
            elif t > end_t:
                break
            total += len(msg.collisions)
    return total

# TODO: Robot on Person collision
# TODO: Person on Robot collision
# TODO: Static obstacle collision
# TODO: Distance to obstacle in front/left/right
