import argparse
import pathlib
from enum import IntEnum
import sys
import yaml

from . import FlexibleBag, get_metrics, global_metric_search
from .parameters import get_all_parameters


class ComputeMode(IntEnum):
    NOTHING = 1
    NEEDED = 2
    EVERYTHING = 3


exception_warnings = set()


def should_compute(name, metric_names, compute_mode, computed_values, saved_names, errors):
    if metric_names is not None and name not in metric_names:
        return False

    if compute_mode == ComputeMode.EVERYTHING:
        return True

    if name in computed_values or name in saved_names or name in errors:
        return False

    return True


def compute_metrics(bag_path, metric_names=None, ignore_errors=False, compute_mode=ComputeMode.NEEDED):
    """Compute all known metrics for the given bag and return results as a dictionary"""
    assert bag_path.is_dir()
    cache_path = bag_path / 'navigation_metrics.yaml'
    if cache_path.exists():
        computed_values = yaml.safe_load(open(cache_path))
    else:
        computed_values = {}
    saved_names = computed_values.pop('saved_names', [])
    errors = computed_values.pop('errors', {})

    computed_values['parameters'] = get_all_parameters(bag_path)

    if compute_mode == ComputeMode.NOTHING:
        return computed_values

    bag = FlexibleBag(bag_path, write_mods=False)

    for name, metric in get_metrics().items():
        if not should_compute(name, metric_names, compute_mode, computed_values, saved_names, errors):
            continue

        try:
            m = metric(bag)
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

        if isinstance(m, dict):
            computed_values.update(m)
            saved_names.append(name)
        else:
            computed_values[name] = m

    output_d = dict(computed_values)
    output_d['saved_names'] = saved_names
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

        if isinstance(value, float):
            v_s = f'{value:.2f}'
        else:
            v_s = str(value)
        print(template.format(name=name, v_s=v_s))

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
