import argparse
import collections
import pathlib
import tabulate
import math

from . import global_metric_search, get_metrics
from .analyze_bag import compute_metrics, ComputeMode
from .dimension import Dimension


def find_bags(folder_path):
    initial = folder_path.resolve()
    if (initial / 'metadata.yaml').exists():
        yield initial

    queue = [initial]
    while queue:
        folder = queue.pop(0)
        for subpath in folder.iterdir():
            if subpath.is_dir():
                if (subpath / 'metadata.yaml').exists():
                    yield subpath
                else:
                    queue.append(subpath)


def analyze_bags(folder_path, compute_mode, metric_names=None):
    data = {}
    for bag_path in find_bags(folder_path):
        metrics = compute_metrics(bag_path, metric_names, ignore_errors=True, compute_mode=compute_mode)
        data[bag_path] = metrics

    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-t', '--table-style', default='tsv')
    parser.add_argument('-c', '--compute-mode', choices=[m.name.lower() for m in ComputeMode], default='needed')
    parser.add_argument('-d', '--dimension', type=Dimension, nargs='?', default='')
    parser.add_argument('-s', '--skip-dimensions', nargs='*')
    parser.add_argument('-i', '--include-dimensions', nargs='*')
    args = parser.parse_args()

    global_metric_search()
    compute_mode = ComputeMode[args.compute_mode.upper()]

    row_sets = collections.defaultdict(list)
    by_metrics = collections.defaultdict(lambda: collections.defaultdict(list))

    my_metrics = None
    if args.skip_dimensions is None and args.include_dimensions is None:
        my_metrics = get_metrics().keys()
    elif args.skip_dimensions is None:
        my_metrics = args.include_dimensions
    else:
        if args.include_dimensions:
            raise RuntimeError("Please don't use include and skip dimensions")
        my_metrics = [metric for metric in get_metrics().keys() if metric not in args.skip_dimensions]

    data = analyze_bags(args.folder, compute_mode, my_metrics)

    base_path = str(args.folder.resolve()) + '/'
    for path, metrics in sorted(data.items()):
        row = {}
        row['name'] = str(path).replace(base_path, '')

        params = row.pop('parameters', {})
        metrics.update(params)
        d_v = args.dimension.get_value(metrics)

        for metric in my_metrics:
            if metric in metrics:
                row[metric] = metrics[metric]
        row_sets[d_v].append(row)

        for metric, value in row.items():
            if isinstance(value, str):
                continue
            by_metrics[d_v][metric].append(value)

    tablefmt = args.table_style
    for d_v, rows in row_sets.items():
        if args.dimension.name:
            print(args.dimension.format_name(d_v))
        print(tabulate.tabulate(rows, headers='keys', tablefmt=tablefmt))
        print()

    for d_v in by_metrics:
        if args.dimension.name:
            print(args.dimension.format_name(d_v))

        rows = []
        for metric, values in by_metrics[d_v].items():
            row = [metric]
            total = 0.0
            the_min = None
            the_max = None
            count = 0
            missing = 0
            for v in values:
                if isinstance(v, bool):
                    v = 1 if v else 0
                elif v is None or (isinstance(v, float) and math.isnan(v)):
                    missing += 1
                    continue
                elif isinstance(v, str):
                    continue

                if count == 0:
                    the_min = v
                    the_max = v
                else:
                    if the_min > v:
                        the_min = v
                    if the_max < v:
                        the_max = v
                total += v
                count += 1

            row += [the_min, the_max, total / max(1, count), total, count, missing]
            rows.append(row)

        print(tabulate.tabulate(rows,
                                headers=['name', 'min', 'max', 'average', 'total', 'count', 'missing'],
                                tablefmt=tablefmt))
        print()


if __name__ == '__main__':
    main()
