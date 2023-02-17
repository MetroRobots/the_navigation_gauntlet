import collections

_nav_metrics = collections.OrderedDict()

RecordedMessage = collections.namedtuple('RecordedMessage', 't msg')


def nav_metric(f):
    """Decorator definition"""
    _nav_metrics[f.__name__] = f


def get_metrics():
    return _nav_metrics
