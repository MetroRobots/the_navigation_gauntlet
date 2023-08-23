import pathlib
import yaml

"""
This module uses yaml files placed in a nested folder structure as a
means to resolving parameters. Let's say you're looking for the parameter
"global_frame" and you have a starting path of /home/gonzo/bagfiles/experiment0/trial0

If the contents of /home/gonzo/bagfiles/experiment0/trial0/metric_params.yaml are
{global_frame: map} then that's the value of the parameter.

If that filepath does not exist, or the parameter name is not present, we move to the
parent directory and look there, i.e. check /home/gonzo/bagfiles/experiment0/metric_params.yaml
and so on until we get to the root directory.
"""

PARAM_FILENAME = 'metric_params.yaml'

_cached_parameters = {}  # To avoid excessive filesystem access,
#                          the contents of the parameter files are cached


def _get_parent_dirs(cur_dir):
    """Iterate over all parent directories (including the starting directory)."""
    folder = cur_dir.resolve()
    while folder:
        yield folder
        if folder.parent == folder:
            return
        else:
            folder = folder.parent


def get_parameter(starting_path, name, default_value=None, namespace=''):
    if not isinstance(starting_path, pathlib.Path):
        starting_path = pathlib.Path(starting_path)
    if not starting_path.is_dir():
        starting_path = starting_path.parent
    for folder in _get_parent_dirs(starting_path):
        if folder not in _cached_parameters:
            filepath = folder / PARAM_FILENAME
            if filepath.exists():
                _cached_parameters[folder] = yaml.safe_load(open(filepath))
            else:
                _cached_parameters[folder] = None
        if not _cached_parameters[folder]:
            continue
        level_params = _cached_parameters[folder]
        if namespace and namespace in level_params:
            if name in level_params[namespace]:
                return level_params[namespace]
        if name in level_params:
            return level_params[name]
    return default_value
