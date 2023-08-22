from navigation_metrics.metric import nav_metric
from navigation_metrics.util import min_max_total_avg, stamp_to_float
import collections


@nav_metric
def compute_time(data):
    start_t = data['/trial_goal_pose'][0].t
    end_t = data['/navigation_result'][0].t

    compute_time_topics = data.get_topics_by_type('benchmark_msgs/msg/ComputeTime')
    values = collections.defaultdict(list)

    for topic in compute_time_topics:
        for bmsg in data[topic]:
            if bmsg.t < start_t:
                continue
            elif bmsg.t > end_t:
                break
            values[bmsg.msg.header.frame_id].append(bmsg)

    metrics = {}
    for name, series in values.items():
        the_min, the_max, total, avg = min_max_total_avg(series, lambda bmsg: stamp_to_float(bmsg.msg.duration))
        metrics[f'total_compute_{name}'] = total
        metrics[f'average_compute_{name}'] = avg
        metrics[f'min_compute_{name}'] = the_min
        metrics[f'max_compute_{name}'] = the_max
        metrics[f'compute_{name}_per_second'] = total / (end_t - start_t)

    return metrics
