import argparse
import collections
import click
from matplotlib.pyplot import subplots, show
import pathlib

from .analyze_bag import ComputeMode
from .analyze_bags import analyze_bags


def get_value(metric_d, name, allow_str=False):
    value = None
    if name in metric_d:
        value = metric_d[name]
    elif name in metric_d.get('parameters', {}):
        value = metric_d['parameters'][name]

    if allow_str or not isinstance(value, str):
        return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x')
    parser.add_argument('y')
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    parser.add_argument('-p', '--plots-axis')
    parser.add_argument('-s', '--series-axis')
    args = parser.parse_args()

    data = analyze_bags(args.folder, ComputeMode.NOTHING)

    xs = collections.defaultdict(lambda: collections.defaultdict(list))
    ys = collections.defaultdict(lambda: collections.defaultdict(list))
    counts = collections.Counter()
    counts[args.x] = 0  # Ensures values are in counter for debug printing
    counts[args.y] = 0

    for path, metrics in sorted(data.items()):
        x_v = get_value(metrics, args.x)
        y_v = get_value(metrics, args.y)
        if args.plots_axis:
            p_v = get_value(metrics, args.plots_axis, allow_str=True)
        else:
            p_v = None
        if args.series_axis:
            s_v = get_value(metrics, args.series_axis, allow_str=True)
        else:
            s_v = None

        if x_v:
            counts[args.x] += 1
        if y_v:
            counts[args.y] += 1
        if p_v:
            counts[args.plots_axis] += 1
        if s_v:
            counts[args.series_axis] += 1

        if x_v is not None and y_v is not None:
            xs[p_v][s_v].append(x_v)
            ys[p_v][s_v].append(y_v)

    for dimension, count in counts.most_common():
        click.secho(f'Dimension "{dimension}" found in {count}/{len(data)} bags', fg='blue' if count else 'red')

    if not xs and not ys:
        return

    num_plots = len(xs)
    fig, ax_v = subplots(num_plots, sharex=True, sharey=True)
    if num_plots == 1:
        axes = [ax_v]
    else:
        axes = ax_v

    for p_v, ax in zip(xs.keys(), axes):
        if args.plots_axis:
            ax.set_title(f'{args.plots_axis} = {p_v}')

        for s_v in xs[p_v]:
            if args.series_axis:
                label = f'{args.series_axis} = {s_v}'
            else:
                label = None

            ax.plot(xs[p_v][s_v], ys[p_v][s_v], 'o', label=label)
        if args.series_axis:
            ax.legend()
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)
    show()


if __name__ == '__main__':
    main()
