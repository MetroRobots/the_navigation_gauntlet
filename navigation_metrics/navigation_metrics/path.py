from tf_transformations import euler_from_quaternion
from angles import shortest_angular_distance

from geometry_msgs.msg import PoseStamped, Point, Pose
from nav_2d_msgs.msg import Pose2DStamped
from std_msgs.msg import Float32

from .metric import RecordedMessage, nav_metric, metric_conversion_function
from .util import pose_stamped_distance, point_distance, metric_min, average, metric_final


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


@metric_conversion_function('/distance_to_goal')
def pose_to_goal_distance(data):
    goal = data['/trial_goal_pose'][0].msg
    seq = []
    for t, msg in data['/path']:
        fmsg = Float32()
        fmsg.data = pose_stamped_distance(goal, msg)
        seq.append(RecordedMessage(t, fmsg))
    return seq


@nav_metric
def distance_to_goal(data):
    return metric_final(data['/distance_to_goal'])


@nav_metric
def angle_to_goal(data):
    goals = data['/trial_goal_pose_2d']
    path = data['/path2d']
    return shortest_angular_distance(goals[0].msg.pose.theta, path[-1].msg.pose.theta)


@nav_metric
def min_distance_to_goal(data):
    return metric_min(data['/distance_to_goal'])


@nav_metric
def avg_distance_to_goal(data):
    return average(data['/distance_to_goal'])


@nav_metric
def path_length(data):
    total = 0.0
    prev_point = None
    for o in data['/path']:
        p = o.msg.pose.position
        if prev_point is None:
            prev_point = p
        total += point_distance(p, prev_point)
        prev_point = p

    return total


@nav_metric
def optimum_efficiency(data):
    pl = path_length(data)
    path = data['/path']
    min_d = pose_stamped_distance(path[-1].msg, path[0].msg)
    return pl / min_d
