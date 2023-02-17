from math import hypot
from tf_transformations import euler_from_quaternion
from angles import shortest_angular_distance

from geometry_msgs.msg import PoseStamped, Point, Pose
from nav_2d_msgs.msg import Pose2DStamped

from .metric import RecordedMessage, nav_metric, metric_conversion_function


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


@metric_conversion_function('/path')
def tf_to_pose(data, period=0.1):
    seq = []
    start_t = data['/trial_goal_pose'][0].t
    end_t = data['/navigation_result'][0].t
    last_t = None

    for t, msg in data['/tf']:
        if t < start_t:
            continue
        elif t > end_t:
            break
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                if last_t is None or (t - last_t) >= period:
                    ps = PoseStamped()
                    ps.header = transform.header
                    ps.pose = transform_to_pose(transform.transform)
                    seq.append(RecordedMessage(t, ps))
                    last_t = t
    return seq


@metric_conversion_function('/path2d')
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


@nav_metric
def min_distance_to_goal(data):
    goal = data['/trial_goal_pose'][0].msg.pose.position
    min_d = None
    for t, msg in data['/path']:
        d = distance(goal, msg.pose.position)
        if min_d is None or min_d > d:
            min_d = d
    return min_d


@nav_metric
def avg_distance_to_goal(data):
    goal = data['/trial_goal_pose'][0].msg.pose.position
    total_d = 0.0
    n = 0
    for t, msg in data['/path']:
        total_d += distance(goal, msg.pose.position)
        n += 1
    return total_d / n


@nav_metric
def path_length(data):
    total = 0.0
    prev_pose = None
    for o in data['/path']:
        p = o.msg.pose.position
        if prev_pose is None:
            prev_pose = p
        total += distance(p, prev_pose)
        prev_pose = p

    return total


@nav_metric
def optimum_efficiency(data):
    pl = path_length(data)
    path = data['/path']
    min_d = distance(path[-1].msg.pose.position, path[0].msg.pose.position)
    return pl / min_d
