import argparse
import pathlib
from gauntlet_analysis.trial_bag import TrialBag
from navigation_metrics import get_metrics, global_metric_search


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    args = parser.parse_args()

    global_metric_search()

    bag = TrialBag(args.bag_path)
    for name, metric in get_metrics().items():
        m = metric(bag)
        print(name, m)


if __name__ == '__main__':
    main()
