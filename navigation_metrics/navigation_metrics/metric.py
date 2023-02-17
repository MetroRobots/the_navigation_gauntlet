import collections

_nav_metrics = collections.OrderedDict()
_conversion_functions = {}

RecordedMessage = collections.namedtuple('RecordedMessage', 't msg')


def nav_metric(f):
    """Decorator definition"""
    _nav_metrics[f.__name__] = f
    return f


def get_metrics():
    return _nav_metrics


def metric_conversion_function(topic):
    """Decorator definition"""
    def actual_decorator(f):
        _conversion_functions[topic] = f
        return f
    return actual_decorator


def get_conversion_functions():
    return _conversion_functions
