import collections
import inspect

_nav_metrics = collections.OrderedDict()
_nav_metric_sets = {}

_parameter_defaults = {}
_parameter_dependencies = {}


# decorator definitions
def nav_metric(f):
    """A nav metric is defined as a function that takes a FlexibleBag as a parameter and
       can return either a simple datatype (in which case the metric's name is assumed to be the same
       as the function's name, or can return a dictionary of metric names to simple datatypes."""
    _nav_metrics[f.__name__] = f

    argspec = inspect.getargspec(f)
    # First parameter is always data
    if len(argspec.args) > 1:
        names = []
        if len(argspec.args) != len(argspec.defaults) + 1:
            raise RuntimeError(f'Function {f.__name__} does not have defaults for parameters specified properly')
        for param_name, param_default in zip(argspec.args[1:], argspec.defaults):
            if param_name in _parameter_defaults and _parameter_defaults[param_name] != param_default:
                raise RuntimeError(f'Function {f.__name__} has a conflicting default value for parameter {param_name}')
            _parameter_defaults[param_name] = param_default
            names.append(param_name)
        _parameter_dependencies[f.__name__] = names
    return f


def nav_metric_set(suffixes):
    """A nav metric set is a function that takes a FlexibleBag as a parameter and returns
       a dictionary of metric names to simple datatypes."""
    # TODO: Check parameters for nav_metric_set
    def inner_decorator(f):
        _nav_metrics[f.__name__] = f
        _nav_metric_sets[f.__name__] = suffixes
        return f
    return inner_decorator


def get_metrics():
    return _nav_metrics


def get_metric_set_info(name):
    return _nav_metric_sets.get(name)


def get_metric_parameter_defaults():
    return _parameter_defaults


def get_parameter_dependencies(name):
    return _parameter_dependencies.get(name, [])


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
