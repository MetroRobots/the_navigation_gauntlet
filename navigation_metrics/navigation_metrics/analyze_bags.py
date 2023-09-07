import argparse
import collections
import pathlib
import tabulate
import math

from . import global_metric_search
from .analyze_bag import compute_metrics


def find_bags(folder_path):
    queue = [folder_path.resolve()]
    while queue:
        folder = queue.pop(0)
        for subpath in folder.glob('*'):
            if subpath.is_dir():
                if (subpath / 'metadata.yaml').exists():
                    yield subpath
                else:
                    queue.append(subpath)


def analyze_bags(folder_path):
    data = {}
    for bag_path in find_bags(folder_path):
        metrics = compute_metrics(bag_path, ignore_errors=True)
        data[bag_path] = metrics
        if len(data) > 5:
            break

    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-s', '--table-style', default='tsv')
    # parser.add_argument('-f', '--force-recompute', action='store_true')
    args = parser.parse_args()

    global_metric_search()
    data = analyze_bags(args.folder)

    by_metrics = collections.defaultdict(list)

    rows = []
    base_path = str(args.folder.resolve()) + '/'
    for path, metrics in sorted(data.items()):
        row = {}
        row['name'] = str(path).replace(base_path, '')
        row.update(metrics)
        rows.append(row)

        for metric, value in metrics.items():
            by_metrics[metric].append(value)

    tablefmt = args.table_style
    print(tabulate.tabulate(rows, headers='keys', tablefmt=tablefmt))
    print()

    rows = []
    for metric, values in by_metrics.items():
        row = [metric]
        total = 0.0
        the_min = None
        the_max = None
        count = 0
        missing = 0
        for v in values:
            if isinstance(v, bool):
                v = 1 if v else 0
            elif v is None or math.isnan(v):
                missing += 1
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

        row += [the_min, the_max, total / count, total, missing]
        rows.append(row)

    print(tabulate.tabulate(rows, headers=['name', 'min', 'max', 'average', 'total', 'missing'], tablefmt=tablefmt))


if __name__ == '__main__':
    main()
