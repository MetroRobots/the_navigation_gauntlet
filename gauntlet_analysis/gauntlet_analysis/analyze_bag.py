import argparse
import pathlib
from gauntlet_analysis.trial_bag import TrialBag
from navigation_metrics import get_metrics, global_metric_search


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    parser.add_argument('-i', '--ignore-errors', action='store_true')
    args = parser.parse_args()

    global_metric_search()

    bag = TrialBag(args.bag_path)
    for name, metric in get_metrics().items():
        try:
            m = metric(bag)
        except Exception:
            if args.ignore_errors:
                continue
            else:
                raise
        print(name, m)


if __name__ == '__main__':
    main()
