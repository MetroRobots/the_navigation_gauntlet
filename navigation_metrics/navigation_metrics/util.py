from math import hypot


def point_distance(p0, p1):
    dx = p0.x - p1.x
    dy = p0.y - p1.y
    dz = p0.z - p1.z
    return hypot(dx, dy, dz)


def pose_distance(p0, p1):
    return point_distance(p0.position, p1.position)


def pose_stamped_distance(p0, p1):
    return point_distance(p0.pose.position, p1.pose.position)
