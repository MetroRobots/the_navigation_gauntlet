from nav_2d_msgs.msg import Twist2DStamped

from .metric import RecordedMessage, nav_metric, metric_conversion_function
from .util import pose2d_distance, stamp_to_float, average, metric_max


@metric_conversion_function('/actual_velocity')
def poses_to_velocity(data):
    seq = []
    prev = None
    for rmsg in data['/path2d']:
        if prev:
            # dt0 = rmsg.t - prev.t
            dt1 = stamp_to_float(rmsg.msg.header.stamp) - stamp_to_float(prev.msg.header.stamp)
            twist = Twist2DStamped()
            twist.header = rmsg.msg.header
            d, turn = pose2d_distance(rmsg.msg.pose, prev.msg.pose)
            twist.velocity.x = d / dt1
            twist.velocity.theta = turn / dt1
            seq.append(RecordedMessage(rmsg.t, twist))
        prev = rmsg
    return seq


def derivative(src_topic):
    seq = []
    prev = None
    for rmsg in src_topic:
        if prev:
            dt1 = stamp_to_float(rmsg.msg.header.stamp) - stamp_to_float(prev.msg.header.stamp)
            twist = Twist2DStamped()
            twist.header = rmsg.msg.header
            v0 = prev.msg.velocity
            v1 = rmsg.msg.velocity
            twist.velocity.x = (v1.x - v0.x) / dt1
            twist.velocity.theta = (v1.theta - v0.theta) / dt1
            seq.append(RecordedMessage(rmsg.t, twist))
        prev = rmsg
    return seq


@metric_conversion_function('/actual_acceleration')
def velocity_to_acceleration(data):
    return derivative(data['/actual_velocity'])


@metric_conversion_function('/actual_jerk')
def acceleration_to_jerk(data):
    return derivative(data['/actual_acceleration'])


@nav_metric
def average_translational_velocity(data):
    return average(data['/actual_velocity'], lambda rmsg: abs(rmsg.msg.velocity.x))


@nav_metric
def average_translational_acceleration(data):
    return average(data['/actual_acceleration'], lambda rmsg: abs(rmsg.msg.velocity.x))


@nav_metric
def average_translational_jerk(data):
    return average(data['/actual_jerk'], lambda rmsg: abs(rmsg.msg.velocity.x))


@nav_metric
def average_rotational_velocity(data):
    return average(data['/actual_velocity'], lambda rmsg: abs(rmsg.msg.velocity.theta))


@nav_metric
def average_rotational_acceleration(data):
    return average(data['/actual_acceleration'], lambda rmsg: abs(rmsg.msg.velocity.theta))


@nav_metric
def average_rotational_jerk(data):
    return average(data['/actual_jerk'], lambda rmsg: abs(rmsg.msg.velocity.theta))


@nav_metric
def max_translational_velocity(data):
    return metric_max(data['/actual_velocity'], lambda rmsg: abs(rmsg.msg.velocity.x))


@nav_metric
def time_not_moving(data, x_threshold=0.1, theta_threshold=0.1):
    total = 0.0
    start = None
    for t, msg in data['/actual_velocity']:
        if msg.velocity.x < x_threshold and msg.velocity.theta < theta_threshold:
            if start is None:
                start = stamp_to_float(msg.header.stamp)
        elif start:
            total += stamp_to_float(msg.header.stamp) - start
            start = None
    return total
