from math import hypot
from angles import shortest_angular_distance
import numpy


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


def default_getter(bmsg):
    return bmsg.msg.data


def average(series, getter=default_getter):
    total = 0.0
    count = 0
    for bmsg in series:
        total += getter(bmsg)
        count += 1
    return total / count


def metric_min(series, getter=default_getter):
    best = None
    for bmsg in series:
        v = getter(bmsg)
        if best is None or v < best:
            best = v
    return best


def metric_max(series, getter=default_getter):
    best = None
    for bmsg in series:
        v = getter(bmsg)
        if best is None or v > best:
            best = v
    return best


def metric_final(series, getter=default_getter):
    bmsg = series[-1]
    return getter(bmsg)


def min_max_total_avg(series, getter=default_getter):
    if not series:
        return None, None, None

    total = 0.0
    the_min = None
    the_max = None
    count = 0
    for bmsg in series:
        v = getter(bmsg)
        if count == 0:
            the_min = v
            the_max = v
        else:
            if the_min > v:
                the_min = v
            if the_max < v:
                the_max = v
        total += v
        count += 1
    return the_min, the_max, total, total / count


def min_max_avg_d(series, getter=default_getter):
    the_min, the_max, _, avg = min_max_total_avg(series, getter)
    return {'min': the_min, 'max': the_max, 'avg': avg}


def min_max_total_avg_d(series, getter=default_getter):
    the_min, the_max, total, avg = min_max_total_avg(series, getter)
    return {'min': the_min, 'max': the_max, 'total': total, 'avg': avg}


def standard_deviation(series, getter=default_getter):
    if not series:
        return 0.0

    values = [getter(bmsg) for bmsg in series]
    n_arr = numpy.array(values)
    return float(numpy.std(n_arr))
