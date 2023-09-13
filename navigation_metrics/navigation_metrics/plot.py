import argparse
import collections
import click
from matplotlib.pyplot import subplots, show
import pathlib
import re

from .analyze_bag import ComputeMode
from .analyze_bags import analyze_bags

MOD_P = re.compile(r'^([\w_]+)(%)(.*)$')


class Dimension:
    def __init__(self, full_name, allow_str=False):
        self.full_name = full_name
        self.allow_str = False
        self.count = 0
        self.alter_fne = None
        self.op = None

        if not self.full_name:
            self.name = ''
            return
        m = MOD_P.match(self.full_name)
        if m:
            self.name, self.op, param = m.groups()
            self.num = float(param)
            self.alter_fne = lambda d: d - d % self.num
        else:
            self.name = self.full_name

    def get_value(self, metric_d):
        if not self.full_name:
            return

        if self.name in metric_d:
            value = metric_d[self.name]
        elif self.name in metric_d.get('parameters', {}):
            value = metric_d['parameters'][self.name]
        else:
            value = None

        if not self.allow_str and isinstance(value, str):
            value = None

        if self.alter_fne and value:
            try:
                value = self.alter_fne(value)
            except Exception:
                raise

        if value is not None:
            self.count += 1

        return value

    def format_name(self, value):
        if not self.name:
            return None
        elif self.op is None:
            return f'{self.name} = {value}'
        elif self.op == '%':
            if value is not None:
                return f'{self.name} ∈ [{value:.2f}, {(value + self.num):.2f}]'
            else:
                return f'{self.name} = {value}'

    def __repr__(self):
        return self.name


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

    dimensions = {
        'x': Dimension(args.x),
        'y': Dimension(args.y),
        'p': Dimension(args.plots_axis, allow_str=True),
        's': Dimension(args.series_axis, allow_str=True),
    }

    for path, metrics in sorted(data.items()):
        values = {d: dimension.get_value(metrics) for d, dimension in dimensions.items()}

        if values['x'] is not None and values['y'] is not None:
            p_v = values['p']
            s_v = values['s']
            xs[p_v][s_v].append(values['x'])
            ys[p_v][s_v].append(values['y'])

    for dimension in dimensions.values():
        if not dimension.full_name:
            continue
        click.secho(f'Dimension "{dimension}" found in {dimension.count}/{len(data)} bags',
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
        ax.set_title(dimensions['p'].format_name(p_v))

        for s_v in sorted(xs[p_v]):
            label = dimensions['s'].format_name(s_v)
            ax.plot(xs[p_v][s_v], ys[p_v][s_v], 'o', label=label)
        if args.series_axis:
            ax.legend()
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)
    show()


if __name__ == '__main__':
    main()
