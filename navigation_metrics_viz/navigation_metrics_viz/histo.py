import argparse
import collections
import click
from matplotlib.pyplot import subplots, show
import pathlib

from navigation_metrics.analyze_bag import ComputeMode
from navigation_metrics.analyze_bags import analyze_bags
from navigation_metrics.dimension import Dimension, matches_any


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x')
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-p', '--plots-axis')
    parser.add_argument('-f', '--filter-axes', nargs='*', default=[])
    parser.add_argument('-n', '--label-count', action='store_true')
    parser.add_argument('-r', '--label-percentage', action='store_true')
    args = parser.parse_args()

    data = analyze_bags(args.folder, ComputeMode.NOTHING)

    counts = collections.defaultdict(collections.Counter)

    dimensions = {
        'x': Dimension(args.x),
        'p': Dimension(args.plots_axis),
    }
    d_filters = []
    for filter_s in args.filter_axes:
        d = Dimension(filter_s)
        d_filters.append(d)

    for path, metrics in sorted(data.items()):
        if d_filters and matches_any(metrics, d_filters):
            continue

        values = {d: dimension.get_value(metrics) for d, dimension in dimensions.items()}

        if values['x'] is not None:
            p_v = values['p']
            counts[p_v][values['x']] += 1

    for dimension in dimensions.values():
        if not dimension.full_name:
            continue
        click.secho(f'Dimension "{dimension}" found in {dimension.count}/{len(data)} bags',
                    fg='blue' if dimension.count else 'red')

    if not counts:
        return

    num_plots = len(counts)
    fig, ax_v = subplots(num_plots, sharex=True, sharey=True)
    if num_plots == 1:
        axes = [ax_v]
    else:
        axes = ax_v

    for p_v, ax in zip(counts.keys(), axes):
        ax.set_title(dimensions['p'].format_name(p_v))

        count = counts[p_v]
        numbers = [k for k in count if isinstance(k, float) or isinstance(k, int)]
        if numbers and dimensions['x'].op == '%':
            low = min(numbers)
            hi = max(numbers)
            xs = []
            width = dimensions['x'].operand
            x = low
            while x <= hi:
                closest = [n for n in numbers if abs(n-x) < width / 100]
                if closest:
                    xs.append(closest[0])
                else:
                    xs.append(x)
                x += width
            ys = [count[x] for x in xs]
            xs = [x + width / 2 for x in xs]
            width *= 0.95
        else:
            try:
                xs = sorted(count.keys())
            except TypeError:
                xs = list(count.keys())
            width = 0.5
            ys = [count[x] for x in xs]

        p = ax.bar(xs, ys, width=width)
        ax.set_xlabel(args.x)
        if args.label_count:
            for label, rect in zip(ys, p):
                if not label:
                    continue
                ax.text(
                    rect.get_x() + rect.get_width() / 2,
                    rect.get_height(),
                    str(label),
                    ha='center',
                    va='top')
        if args.label_percentage:
            N = sum(ys)
            print(N, dimensions['x'].count)
            for label, rect in zip(ys, p):
                if not label:
                    continue
                ax.text(
                    rect.get_x() + rect.get_width() / 2,
                    rect.get_height(),
                    f'{label*100/N:.1f}%',
                    ha='center',
                    va='bottom')

    show()


if __name__ == '__main__':
    main()
