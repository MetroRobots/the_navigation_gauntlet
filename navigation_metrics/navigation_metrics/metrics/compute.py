from navigation_metrics.metric import nav_metric
from navigation_metrics.util import min_max_total_avg_d, stamp_to_float
import collections


@nav_metric
def compute_time(data):
    compute_time_topics = data.get_topics_by_type('benchmark_msgs/msg/ComputeTime')
    values = collections.defaultdict(list)

    for topic in compute_time_topics:
        for bmsg in data[topic]:
            values[bmsg.msg.header.frame_id].append(bmsg)

    metrics = {}
    for name, series in values.items():
        s_metrics = min_max_total_avg_d(series, lambda bmsg: stamp_to_float(bmsg.msg.duration))
        for k, v in s_metrics.items():
            metrics[f'{name}/{k}'] = v
        metrics[f'{name}/per_second'] = s_metrics['total'] / data.length()

    return metrics
