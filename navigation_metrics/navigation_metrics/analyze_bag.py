import argparse
import pathlib
from . import FlexibleBag, MissingTopicException, get_metrics, global_metric_search


def compute_metrics(bag_path, ignore_errors=False):
    """Compute all known metrics for the given bag and return results as a dictionary"""
    bag = FlexibleBag(bag_path, read_everything=True)
    computed_values = {}

    for name, metric in get_metrics().items():
        try:
            m = metric(bag)
        except MissingTopicException as e:
            print(f'Missing data: {e} for {name}')
            continue
        except Exception as e:
            if ignore_errors:
                computed_values[name] = str(e)
                continue
            else:
                raise

        if isinstance(m, dict):
            computed_values.update(m)
        else:
            computed_values[name] = m
    return computed_values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    parser.add_argument('-i', '--ignore-errors', action='store_true')
    args = parser.parse_args()

    global_metric_search()

    metrics = compute_metrics(args.bag_path, args.ignore_errors)
    if not metrics:
        print('No metrics computed.')
        return

    max_l = max(len(k) for k in metrics.keys())
    template = '{name:' + str(max_l + 1) + 's} {v_s:8}'
    for name, value in metrics.items():
        if isinstance(value, float):
            v_s = f'{value:.2f}'
        else:
            v_s = str(value)
        print(template.format(name=name, v_s=v_s))


if __name__ == '__main__':
    main()
