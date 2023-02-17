from math import hypot

from geometry_msgs.msg import PoseStamped, Point, Pose

from .metric import nav_metric


def distance(p0, p1):
    dx = p0.x - p1.x
    dy = p0.y - p1.y
    dz = p0.z - p1.z
    return hypot(dx, dy, dz)


def vector_to_point(v):
    p = Point()
    p.x = v.x
    p.y = v.y
    p.z = v.z
    return p


def transform_to_pose(transform):
    pose = Pose()
    pose.position = vector_to_point(transform.translation)
    pose.orientation = transform.rotation
    return pose


def tf_to_pose(data):
    seq = []
    for t, msg in data['/tf']:
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                ps = PoseStamped()
                ps.header = transform.header
                ps.pose = transform_to_pose(transform.transform)
                seq.append((t, ps))
    return seq


@nav_metric
def distance_to_goal(data):
    goals = data['/trial_goal_pose']
    path = data['/path']
    final_pose = path[-1][1]

    return distance(goals[0][1].pose.position, final_pose.pose.position)
