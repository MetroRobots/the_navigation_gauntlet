import argparse
import collections
import click
from matplotlib.pyplot import subplots, show
import pathlib
import numpy

from navigation_metrics.analyze_bag import ComputeMode
from navigation_metrics.analyze_bags import analyze_bags
from navigation_metrics.dimension import Dimension, matches_any


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x')
    parser.add_argument('y')
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-p', '--plots-axis')
    parser.add_argument('-s', '--series-axis')
    parser.add_argument('-f', '--filter-axes', nargs='*', default=[])
    parser.add_argument('-l', '--log', action='store_true')
    parser.add_argument('--horizontal-lines', type=float, nargs='+', default=[])
    args = parser.parse_args()

    data = analyze_bags(args.folder, ComputeMode.NOTHING)

    xs = collections.defaultdict(lambda: collections.defaultdict(list))
    ys = collections.defaultdict(lambda: collections.defaultdict(list))
    counts = collections.Counter()

    dimensions = {
        'x': Dimension(args.x),
        'y': Dimension(args.y),
        'p': Dimension(args.plots_axis),
        's': Dimension(args.series_axis),
    }
    d_filters = []
    for filter_s in args.filter_axes:
        d = Dimension(filter_s)
        d_filters.append(d)

    N = 0
    for path, metrics in sorted(data.items()):
        if d_filters and matches_any(metrics, d_filters):
            continue

        N += 1
        values = {d: dimension.get_value(metrics) for d, dimension in dimensions.items()}

        if values['x'] is not None and values['y'] is not None:
            p_v = values['p']
            s_v = values['s']
            counts[p_v] += 1
            xs[p_v][s_v].append(values['x'])
            ys[p_v][s_v].append(values['y'])

    for d_filter in d_filters:
        click.secho(f'Filter dimension "{d_filter}" found in {d_filter.count}/{len(data)} bags',
                    fg='blue' if d_filter.count else 'red')

    for dimension in dimensions.values():
        if not dimension.full_name:
            continue
        click.secho(f'Dimension "{dimension}" found in {dimension.count}/{N} bags',
                    fg='blue' if dimension.count else 'red')

    if not xs and not ys:
        return

    num_plots = len(xs)
    fig, ax_v = subplots(num_plots, sharex=True, sharey=True)
    if num_plots == 1:
        axes = [ax_v]
    else:
        axes = ax_v

    for p_v, ax in zip(xs.keys(), axes):
        if args.log:
            ax.set_xscale('log')
        title = dimensions['p'].format_name(p_v)
        if title:
            ax.set_title(f'{title} (N={counts[p_v]})')
        else:
            ax.set_title(f'N={counts[p_v]}')

        try:
            ordered_s = sorted(xs[p_v])
        except TypeError:
            ordered_s = list(xs[p_v])
        for s_v in ordered_s:
            label = dimensions['s'].format_name(s_v)

            if dimensions['x'].op == '%':
                buckets = collections.defaultdict(list)
                for x, y in zip(xs[p_v][s_v], ys[p_v][s_v]):
                    buckets[x].append(y)
                bxs = sorted(buckets)
                bys = []
                berr = []
                for bx in bxs:
                    values = numpy.array(buckets[bx])
                    bys.append(numpy.mean(values))
                    berr.append(numpy.std(values))
                ax.errorbar(bxs, bys, yerr=berr, fmt='o', label=label)

            else:
                ax.plot(xs[p_v][s_v], ys[p_v][s_v], 'o', label=label)
        if args.series_axis:
            ax.legend()
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)

        for y_val in args.horizontal_lines:
            ax.axhline(y=y_val, color='r', linestyle='-')
    show()


if __name__ == '__main__':
    main()
