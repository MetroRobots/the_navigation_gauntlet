from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import BagMessage, flexible_bag_converter_function
from navigation_metrics.util import average

from angles import shortest_angular_distance
from math import atan2, pi
from std_msgs.msg import Float32


@flexible_bag_converter_function('/direction_of_travel_coefficient')
def direction_of_travel_coefficient(data):
    seq = []
    for bmsg0, bmsg1 in data['/path2d', '/actual_velocity']:
        pose_yaw = bmsg0.msg.pose.theta
        vel_yaw = atan2(bmsg1.msg.velocity.y, bmsg1.msg.velocity.x)

        fmsg = Float32()
        fmsg.data = 1.0 - abs(shortest_angular_distance(pose_yaw, vel_yaw)) / pi

        seq.append(BagMessage(bmsg0.t, fmsg))
    return seq


@nav_metric
def direction_of_travel_factor(data):
    return average(data['/direction_of_travel_coefficient'])
