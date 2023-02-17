import argparse
import pathlib
from gauntlet_analysis.trial_bag import TrialBag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    args = parser.parse_args()

    bag = TrialBag(args.bag_path)
    print(bag)


if __name__ == '__main__':
    main()
