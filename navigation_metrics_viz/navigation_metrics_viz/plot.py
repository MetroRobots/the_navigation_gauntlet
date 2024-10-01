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
    parser.add_argument('--linear', action='store_true')
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
            plot_x = xs[p_v][s_v]
            plot_y = ys[p_v][s_v]

            if dimensions['x'].op != '%':
                ax.plot(plot_x, plot_y, 'o', label=label)
            else:
                buckets = collections.defaultdict(list)
                for x, y in zip(plot_x, plot_y):
                    buckets[x].append(y)
                plot_x = sorted(buckets)
                plot_y = []
                y_err = []
                for x_v in plot_x:
                    values = numpy.array(buckets[x_v])
                    plot_y.append(numpy.mean(values))
                    y_err.append(numpy.std(values))
                ax.errorbar(plot_x, plot_y, yerr=y_err, fmt='o', label=label)

            if args.linear:
                m, b = numpy.polyfit(plot_x, plot_y, 1)
                ax.plot(plot_x, m*numpy.array(plot_x)+b, label=f'y = {m:.2f}x + {b:.2f}')
        if args.series_axis or args.linear:
            ax.legend()
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)

        for y_val in args.horizontal_lines:
            ax.axhline(y=y_val, color='r', linestyle='-')
    show()


if __name__ == '__main__':
    main()
