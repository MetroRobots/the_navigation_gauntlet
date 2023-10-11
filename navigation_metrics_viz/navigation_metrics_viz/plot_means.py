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
    parser.add_argument('metric')
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-p', '--plots-axis')
    parser.add_argument('-s', '--series-axis')
    parser.add_argument('-f', '--filter-axes', nargs='*', default=[])
    parser.add_argument('-m', '--plot-max', action='store_true')
    parser.add_argument('-x', '--marker-scale', type=int, default=1)
    parser.add_argument('-y')
    args = parser.parse_args()

    data = analyze_bags(args.folder, ComputeMode.NOTHING)

    ys = collections.defaultdict(lambda: collections.defaultdict(list))
    errors = collections.defaultdict(lambda: collections.defaultdict(list))
    maxes = collections.defaultdict(lambda: collections.defaultdict(list))
    added_values = collections.defaultdict(lambda: collections.defaultdict(list))
    counts = collections.Counter()

    dimensions = {
        'y': Dimension(f'{args.metric}/avg'),
        'e': Dimension(f'{args.metric}/stddev'),
        'p': Dimension(args.plots_axis),
        's': Dimension(args.series_axis),
        'm': Dimension(f'{args.metric}/max'),
        'a': Dimension(args.y)
    }
    d_filters = []
    for filter_s in args.filter_axes:
        d = Dimension(filter_s)
        d_filters.append(d)

    N = 0
    for path, metrics in sorted(data.items(), key=lambda d: dimensions['y'].get_value(d[1])):
        if d_filters and matches_any(metrics, d_filters):
            continue

        N += 1
        values = {d: dimension.get_value(metrics) for d, dimension in dimensions.items()}

        if values['y'] is not None and values['e'] is not None:
            p_v = values['p']
            s_v = values['s']
            counts[p_v] += 1

            errors[p_v][s_v].append(values['e'])
            ys[p_v][s_v].append(values['y'])
            maxes[p_v][s_v].append(values['m'])
            added_values[p_v][s_v].append(values['a'])

    for d_filter in d_filters:
        click.secho(f'Filter dimension "{d_filter}" found in {d_filter.count}/{len(data)} bags',
                    fg='blue' if d_filter.count else 'red')

    for dimension in dimensions.values():
        if not dimension.full_name:
            continue
        click.secho(f'Dimension "{dimension}" found in {dimension.count}/{N} bags',
                    fg='blue' if dimension.count else 'red')

    if not ys:
        return

    num_plots = len(ys)
    fig, ax_v = subplots(num_plots, sharex=True, sharey=True)
    if num_plots == 1:
        axes = [ax_v]
    else:
        axes = ax_v

    for p_v, ax in zip(ys.keys(), axes):
        ax.grid(axis='y')

        title = dimensions['p'].format_name(p_v)
        if title:
            ax.set_title(f'{title} (N={counts[p_v]})')
        else:
            ax.set_title(f'N={counts[p_v]}')

        try:
            ordered_s = sorted(ys[p_v])
        except TypeError:
            ordered_s = list(ys[p_v])
        for s_v in ordered_s:
            label = dimensions['s'].format_name(s_v)
            d = ys[p_v][s_v]
            x = list(range(len(d)))
            ax.errorbar(x, d, yerr=errors[p_v][s_v], fmt='o', label=label)
            if args.plot_max:
                ax.plot(x, maxes[p_v][s_v], '_', label=dimensions['m'], markersize=args.marker_scale)
            if args.y:
                ax.plot(x, added_values[p_v][s_v], 'x', label=dimensions['a'], markersize=args.marker_scale)
        if args.series_axis or args.plot_max or args.y:
            ax.legend()
        ax.set_xlabel('')
        ax.set_ylabel(args.metric)
    show()


if __name__ == '__main__':
    main()
