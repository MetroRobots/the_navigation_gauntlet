import argparse
import pathlib
from enum import IntEnum
import sys
import yaml

from . import FlexibleBag, get_metrics, global_metric_search, MissingTopicException
from .metric import get_metric_parameter_defaults, get_parameter_dependencies
from .parameters import get_all_parameters
from .window import WindowBag, TimeWindow


class ComputeMode(IntEnum):
    NOTHING = 1
    NEEDED = 2
    EVERYTHING = 3
    CLEAN = 4


exception_warnings = set()


def should_compute(name, metric_names, compute_mode, computed_values, errors, cached_parameters):
    if metric_names is not None and name not in metric_names:
        return False

    if compute_mode == ComputeMode.EVERYTHING or compute_mode == ComputeMode.CLEAN:
        return True

    for param_name in get_parameter_dependencies(name):
        if computed_values['parameters'].get(param_name) != cached_parameters.get(param_name):
            return True

    short_name = name.partition('/')[0]

    if short_name in computed_values or short_name in errors:
        return False

    return True


def get_standard_window(bag):
    try:
        starts = bag['/trial_goal_pose']
        ends = bag['/navigation_result']

        if not starts or not ends:
            return

        return TimeWindow(starts[0].t, ends[0].t)
    except MissingTopicException:
        pass


def compute_metrics(bag_path, metric_names=None, ignore_errors=False, compute_mode=ComputeMode.NEEDED):
    """Compute all known metrics for the given bag and return results as a dictionary"""
    assert bag_path.is_dir()
    cache_path = bag_path / 'navigation_metrics.yaml'
    if cache_path.exists() and compute_mode != ComputeMode.CLEAN:
        computed_values = yaml.safe_load(open(cache_path))
    else:
        computed_values = {}
    errors = computed_values.pop('errors', {})
    cached_parameters = computed_values.pop('parameters', {})

    computed_values['parameters'] = get_all_parameters(bag_path)
    for k, v in get_metric_parameter_defaults().items():
        if k not in computed_values['parameters']:
            computed_values['parameters'][k] = v

    if compute_mode == ComputeMode.NOTHING:
        return computed_values

    bag = FlexibleBag(bag_path, write_mods=False)

    window = get_standard_window(bag)
    if not window:
        return computed_values

    data = WindowBag(bag, window)

    for name, metric in get_metrics().items():
        if not should_compute(name, metric_names, compute_mode, computed_values, errors, cached_parameters):
            continue

        metric_params = {}
        for param_name in get_parameter_dependencies(name):
            metric_params[param_name] = computed_values['parameters'][param_name]
        try:
            m = metric(data, **metric_params)

            if name in errors:
                del errors[name]
        except Exception as e:
            if ignore_errors:
                error_s = f'{e} for {name}'
                if error_s not in exception_warnings:
                    exception_warnings.add(error_s)
                    print(error_s, file=sys.stderr)
                errors[name] = str(e)
                continue
            else:
                raise

        computed_values[name] = m

    output_d = dict(computed_values)
    if errors:
        output_d['errors'] = errors
    yaml.safe_dump(output_d, open(cache_path, 'w'))

    return computed_values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    parser.add_argument('-i', '--ignore-errors', action='store_true')
    parser.add_argument('-c', '--compute-mode', choices=[m.name.lower() for m in ComputeMode], default='needed')
    args = parser.parse_args()

    global_metric_search()

    compute_mode = ComputeMode[args.compute_mode.upper()]
    metrics = compute_metrics(args.bag_path, ignore_errors=args.ignore_errors, compute_mode=compute_mode)
    if not metrics:
        print('No metrics computed.')
        return

    max_l = max(len(k) for k in metrics.keys())
    template = '{name:' + str(max_l + 1) + 's} {v_s:8}'
    for name, value in metrics.items():
        if name == 'parameters':
            continue

        def format_value(v):
            if isinstance(v, float):
                return f'{v:.2f}'
            else:
                return str(v)

        if isinstance(value, dict):
            for k, v in value.items():
                print(template.format(name=f'{name}/{k}', v_s=format_value(v)))
            continue
        else:
            print(template.format(name=name, v_s=format_value(value)))

    if 'parameters' in metrics:
        print('\nParameters:')
        for name, value in metrics['parameters'].items():
            if isinstance(value, float):
                v_s = f'{value:.2f}'
            else:
                v_s = str(value)
            print(template.format(name=name, v_s=v_s))


if __name__ == '__main__':
    main()
