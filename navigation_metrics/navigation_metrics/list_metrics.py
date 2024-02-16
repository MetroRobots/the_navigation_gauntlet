import argparse
import click
from . import get_metrics, global_metric_search


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    global_metric_search()

    for metric_name, metric_fne in get_metrics().items():
        click.secho(f'{metric_name} ', fg='bright_white', nl=False)
        click.secho(' (from ', nl=False)
        click.secho(metric_fne.__module__, fg='green', nl=False)
        click.secho(')')

        if metric_fne.__doc__:
            lines = [line.strip() for line in metric_fne.__doc__.split('\n')]
            while lines and not lines[0]:
                lines.pop(0)
            while lines and not lines[-1]:
                lines.pop()

            if not args.verbose:
                lines = lines[:1]
            for line in lines:
                click.secho(f'\t{line}', fg='cyan')

        click.secho('')
