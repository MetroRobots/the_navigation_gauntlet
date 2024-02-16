from std_msgs.msg import Float32

from polygon_utils.shapely_lib import polygon_from_msg, point_from_msg

from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import BagMessage, flexible_bag_converter_function
from navigation_metrics.util import min_max_avg_d


@flexible_bag_converter_function('/obstacle_clearance')
def calculate_obstacle_clearance(data):
    polygons_msg = data['/polygon_map'][0].msg
    spolygons = [polygon_from_msg(poly) for poly in polygons_msg.polygons]

    seq = []
    for t, msg in data['/path']:
        spoint = point_from_msg(msg.pose.position)
        ds = [spoly.exterior.distance(spoint) for spoly in spolygons]
        fmsg = Float32()
        fmsg.data = min(ds)
        seq.append(BagMessage(t, fmsg))
    return seq


@nav_metric
def clearing_distance(data):
    return min_max_avg_d(data['/obstacle_clearance'])


@nav_metric
def total_collisions(data):
    """
    The number of collisions detected.

    Units: Count
    Topic Types: collision_log_msgs/NamedCollisions
    """
    collision_topics = data.get_topics_by_type('collision_log_msgs/msg/NamedCollisions')
    total = 0
    for topic in collision_topics:
        for t, msg in data[topic]:
            total += len(msg.collisions)
    return total

# TODO: Robot on Person collision
# TODO: Person on Robot collision
# TODO: Static obstacle collision
# TODO: Distance to obstacle in front/left/right
