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


def find_downstream_dependencies(target_package_name='navigation_metrics'):
    """Return all packages that have a build_export_depend on the target package."""
    import os
    from ament_index_python.packages import get_packages_with_prefixes
    from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package

    for package_name, package_path in get_packages_with_prefixes().items():
        package_file_path = os.path.join(package_path, 'share', package_name, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(package_file_path):
            # only try to import catkin if a PACKAGE_FILE is found
            continue

        try:
            package = parse_package(package_file_path)
        except InvalidPackage as e:
            print('Could not parse package file "%s":\n%s' % (package_file_path, e))
            continue

        for dep in package.build_export_depends:
            if dep.name == target_package_name:
                yield package.name
                break


def global_metric_search():
    """Load all the navigation_metrics, including those from downstream packages.

    This call looks for packages that depend on this package, and then imports them,
    in order to populate the metrics and conversion functions datastructures.
    """
    for package_name in find_downstream_dependencies():
        try:
            __import__(package_name)
        except ModuleNotFoundError:
            pass
