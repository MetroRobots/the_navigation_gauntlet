# The Navigation Gauntlet

An open framework for testing navigation algorithms using different robots and different simulators.

## Metrics
To analyze a bag file, run

    ros2 run navigation_metrics analyze_bag path/to/bag

To add metrics of your own, simply implement a function that takes a [`FlexibleBag`](navigation_metrics/navigation_metrics/flexible_bag.py) as a parameter, and return the value of your metric, e.g.

```python
from navigation_metrics.metric import nav_metric

@nav_metric
def total_collisions(data):
    collision_topics = data.get_topics_by_type('collision_msgs/msg/Collisions')
    total = 0
    for topic in collision_topics:
        for t, msg in data[topic]:
            total += len(msg.collisions)
    return total
```
