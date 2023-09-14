import argparse
import collections
from matplotlib.pyplot import subplots, show
import pathlib

from navigation_metrics import FlexibleBag


class Datum:
    def __init__(self, full_name):
        self.full_name = full_name
        parts = self.full_name.split('.')
        self.topic = parts[0]
        self.subfields = parts[1:]

    def get_sequence(self, bag):
        if self.subfields == ['t']:
            t0 = bag.get_start_time()
            return [x.t - t0 for x in bag[self.topic]]
        seq = []
        for t, msg in bag[self.topic]:
            o = msg
            for subfield in self.subfields:
                o = getattr(o, subfield)
            seq.append(o)
        return seq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfiles', metavar='bagfile', type=pathlib.Path, nargs='+')
    parser.add_argument('-x')
    parser.add_argument('-y')
    args = parser.parse_args()

    xs = collections.defaultdict(lambda: collections.defaultdict(list))
    ys = collections.defaultdict(lambda: collections.defaultdict(list))

    datums = [Datum(args.x), Datum(args.y)]

    for bag_path in args.bagfiles:
        bag = FlexibleBag(bag_path, write_mods=False)
        x, y = [datum.get_sequence(bag) for datum in datums]
        xs[None][bag_path.stem] = x
        ys[None][bag_path.stem] = y

    num_plots = len(xs)
    fig, ax_v = subplots(num_plots, sharex=True, sharey=True)
    if num_plots == 1:
        axes = [ax_v]
    else:
        axes = ax_v

    for p_v, ax in zip(xs.keys(), axes):
        for s_v in sorted(xs[p_v]):
            ax.plot(xs[p_v][s_v], ys[p_v][s_v], '.-', label=str(s_v))
        if len(args.bagfiles) > 1:
            ax.legend()
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)
    show()


if __name__ == '__main__':
    main()
