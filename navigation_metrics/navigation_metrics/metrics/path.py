from math import atan2

from tf_transformations import euler_from_quaternion
from angles import shortest_angular_distance

from geometry_msgs.msg import PoseStamped, Point, Pose, Pose2D
from nav_2d_msgs.msg import Path2D, Pose2DStamped
from std_msgs.msg import Float32

from polygon_utils import make_point
from polygon_utils.shortest_path import shortest_path

from navigation_metrics.metric import nav_metric, nav_metric_set
from navigation_metrics.flexible_bag import BagMessage, flexible_bag_converter_function
from navigation_metrics.util import pose_stamped_distance, pose2d_distance, point_distance, metric_final
from navigation_metrics.util import min_max_avg_d


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


@flexible_bag_converter_function('/path')
def tf_to_pose(data):
    seq = []
    last_t = None

    global_frame = data.get_parameter('global_frame', 'map')
    robot_frame = data.get_parameter('robot_frame', 'base_link')
    period = data.get_parameter('path_point_period', 0.1)

    for t, msg in data['/tf']:
        for transform in msg.transforms:
            if transform.header.frame_id == global_frame and transform.child_frame_id == robot_frame:
                if last_t is None or (t - last_t) >= period:
                    ps = PoseStamped()
                    ps.header = transform.header
                    ps.pose = transform_to_pose(transform.transform)
                    seq.append(BagMessage(t, ps))
                    last_t = t
    return seq


def pose_to_pose2d(msg):
    pose2d = Pose2DStamped()
    pose2d.header = msg.header
    pose2d.pose.x = msg.pose.position.x
    pose2d.pose.y = msg.pose.position.y
    quat = msg.pose.orientation
    qa = quat.x, quat.y, quat.z, quat.w
    angles = euler_from_quaternion(qa)
    pose2d.pose.theta = angles[-1]
    return pose2d


@flexible_bag_converter_function('/path2d')
def convert_pose_to_pose2d(data):
    seq = []
    for t, msg in data['/path']:
        pose2d = pose_to_pose2d(msg)
        seq.append(BagMessage(t, pose2d))
    return seq


@flexible_bag_converter_function('/trial_goal_pose_2d')
def convert_goal_pose_to_pose2d(data):
    seq = []
    for t, msg in data['/trial_goal_pose']:
        pose2d = pose_to_pose2d(msg)
        seq.append(BagMessage(t, pose2d))
    return seq


@flexible_bag_converter_function('/distance_to_goal')
def pose_to_goal_distance(data):
    goal = data['/trial_goal_pose'][0].msg
    seq = []
    for t, msg in data['/path']:
        fmsg = Float32()
        fmsg.data = pose_stamped_distance(goal, msg)
        seq.append(BagMessage(t, fmsg))
    return seq


@nav_metric
def angle_to_goal(data):
    """
    Angle to the goal at the end of the trial

    Units: Radians
    Topics: /trial_goal_pose_2d /path2d
    """
    goals = data['/trial_goal_pose_2d']
    path = data['/path2d']
    return abs(shortest_angular_distance(goals[0].msg.pose.theta, path[-1].msg.pose.theta))


@nav_metric_set(['min', 'max', 'avg', 'final'])
def distance_to_goal(data):
    """
    The minimum/maximum/average/final distance to the goal.

    Units: meters
    Topics: /distance_to_goal
    """
    distances = data['/distance_to_goal']
    metrics = min_max_avg_d(distances)
    metrics['final'] = metric_final(distances)
    return metrics


@nav_metric
def path_length(data):
    """
    Length of the path taken over the entire trial.

    Units: Meters
    Topics: /path
    """
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
def straight_line_efficiency(data):
    """
    The ratio of the straight line connecting the beginning and ending of the path to the actual path length

    Units: Float [0, 1]
    Topics: /path
    """
    pl = path_length(data)
    path = data['/path']
    min_d = pose_stamped_distance(path[-1].msg, path[0].msg)
    return min_d / pl


def interpolate_path(start_pose, path, goal_pose):
    path_msg = Path2D()
    path_msg.header = start_pose.header

    path_msg.poses.append(pose_to_pose2d(start_pose).pose)

    for point, next_point in zip(path, path[1:]):
        dx = next_point.x - point.x
        dy = next_point.y - point.y
        pose = Pose2D()
        pose.x = point.x
        pose.y = point.y
        pose.theta = atan2(dy, dx)

        path_msg.poses.append(pose)

    straight_last_pose = Pose2D()
    straight_last_pose.x = path[-1].x
    straight_last_pose.y = path[-1].y
    straight_last_pose.theta = path_msg.poses[-1].theta
    path_msg.poses.append(straight_last_pose)

    path_msg.poses.append(pose_to_pose2d(goal_pose).pose)
    return path_msg


def shortest_path_calculation(data):
    start_bmsg = data['/path'][0]
    start_pose = start_bmsg.msg
    end_pose = data['/path'][-1].msg

    start_pt = make_point(start_pose.pose.position.x, start_pose.pose.position.y)
    goal_pt = make_point(end_pose.pose.position.x, end_pose.pose.position.y)

    if '/polygon_map' in data:
        polygons = data['/polygon_map'][0].msg
        path2d = shortest_path(polygons, start_pt, goal_pt)
    else:
        path2d = [start_pt, goal_pt]

    path = interpolate_path(start_pose, path2d, end_pose)

    return path


@nav_metric
def efficiency(data):
    """
    The ratio of the shortest possible path (given obstacles) to the actual path length

    Units: Float [0, 1]
    Topics: /path, /polygon_map
    """
    pl = path_length(data)

    op = shortest_path_calculation(data)
    od = 0.0
    prev_pose = None
    for pose in op.poses:
        if prev_pose is None:
            prev_pose = pose
        d, _ = pose2d_distance(pose, prev_pose)
        od += d
        prev_pose = pose

    return od / pl
