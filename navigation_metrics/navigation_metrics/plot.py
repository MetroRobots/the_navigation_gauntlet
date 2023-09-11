import argparse
import click
from matplotlib.pyplot import subplots, show
import pathlib

from .analyze_bag import ComputeMode
from .analyze_bags import analyze_bags


def get_value(metric_d, name):
    value = None
    if name in metric_d:
        value = metric_d[name]
    elif name in metric_d.get('parameters', {}):
        value = metric_d['parameters'][name]

    if not isinstance(value, str):
        return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x')
    parser.add_argument('y')
    parser.add_argument('folder', type=pathlib.Path, default='.', nargs='?')
    args = parser.parse_args()

    data = analyze_bags(args.folder, ComputeMode.NOTHING)

    xs = []
    ys = []

    for path, metrics in sorted(data.items()):
        x_v = get_value(metrics, args.x)
        y_v = get_value(metrics, args.y)
        if x_v is not None and y_v is not None:
            xs.append(x_v)
            ys.append(y_v)

    click.secho(f'Dimension "{args.x}" found in {len(xs)}/{len(data)} bags', fg='blue' if xs else 'red')
    click.secho(f'Dimension "{args.y}" found in {len(ys)}/{len(data)} bags', fg='blue' if ys else 'red')

    if not xs and not ys:
        return
    fig, axes = subplots()
    axes.plot(xs, ys, 'o')
    axes.set_xlabel(args.x)
    axes.set_ylabel(args.y)
    show()


if __name__ == '__main__':
    main()
