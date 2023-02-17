from math import hypot
from angles import shortest_angular_distance


def point_distance(p0, p1):
    dx = p0.x - p1.x
    dy = p0.y - p1.y
    dz = p0.z - p1.z
    return hypot(dx, dy, dz)


def pose_distance(p0, p1):
    return point_distance(p0.position, p1.position)


def pose_stamped_distance(p0, p1):
    return point_distance(p0.pose.position, p1.pose.position)


def pose2d_distance(p0, p1):
    dx = p0.x - p1.x
    dy = p0.y - p1.y
    return hypot(dx, dy), shortest_angular_distance(p0.theta, p1.theta)


def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def average(series, getter):
    total = 0.0
    count = 0
    for rmsg in series:
        total += getter(rmsg)
        count += 1
    return total / count


def metric_max(series, getter):
    best = None
    for rmsg in series:
        v = getter(rmsg)
        if best is None or v > best:
            best = v
    return best
