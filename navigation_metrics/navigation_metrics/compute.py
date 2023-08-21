from .metric import nav_metric

import collections


@nav_metric
def compute_time(data):
    start_t = data['/trial_goal_pose'][0].t
    end_t = data['/navigation_result'][0].t

    compute_time_topics = data.get_topics_by_type('benchmark_msgs/msg/ComputeTime')
    values = collections.defaultdict(list)
    counts = collections.Counter()

    for topic in compute_time_topics:
        for t, msg in data[topic]:
            if t < start_t:
                continue
            elif t > end_t:
                break
            name = msg.header.frame_id
            d_s = msg.duration.sec + msg.duration.nanosec / 1e9
            values[name].append(d_s)
            counts[name] += 1

    metrics = {}
    for name, series in values.items():
        total = sum(series)
        metrics[f'total_compute_{name}'] = total
        metrics[f'average_compute_{name}'] = total / counts[name]
        metrics[f'min_compute_{name}'] = min(series)
        metrics[f'max_compute_{name}'] = max(series)
        metrics[f'compute_{name}_per_second'] = total / (end_t - start_t)

    return metrics
