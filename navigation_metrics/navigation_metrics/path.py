from math import hypot
from tf_transformations import euler_from_quaternion
from angles import shortest_angular_distance

from geometry_msgs.msg import PoseStamped, Point, Pose
from nav_2d_msgs.msg import Pose2DStamped

from .metric import RecordedMessage, nav_metric


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
                seq.append(RecordedMessage(t, ps))
    return seq


def pose_to_pose2d(data):
    seq = []
    for t, msg in data['/path']:
        pose2d = Pose2DStamped()
        pose2d.header = msg.header
        pose2d.pose.x = msg.pose.position.x
        pose2d.pose.y = msg.pose.position.y
        quat = msg.pose.orientation
        qa = quat.x, quat.y, quat.z, quat.w
        angles = euler_from_quaternion(qa)
        pose2d.pose.theta = angles[-1]
        seq.append(RecordedMessage(t, pose2d))
    return seq


@nav_metric
def distance_to_goal(data):
    goals = data['/trial_goal_pose']
    path = data['/path']

    return distance(goals[0].msg.pose.position, path[-1].msg.pose.position)


@nav_metric
def angle_to_goal(data):
    goals = data['/trial_goal_pose_2d']
    path = data['/path2d']
    return shortest_angular_distance(goals[0].msg.pose.theta, path[-1].msg.pose.theta)
