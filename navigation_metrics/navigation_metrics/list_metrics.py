import argparse
import click
from . import get_metrics, global_metric_search
from .metric import get_metric_set_info, get_parameter_dependencies, get_metric_parameter_defaults


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-d', '--debug', action='store_true')
    args = parser.parse_args()

    global_metric_search()

    missing = []
    param_defaults = get_metric_parameter_defaults()

    for metric_name, metric_fne in get_metrics().items():
        set_info = get_metric_set_info(metric_name)
        if set_info is None:
            metric_names = [metric_name]
        else:
            metric_names = [f'{metric_name}/{suffix}' for suffix in set_info]

        for mn in metric_names:
            click.secho(f'{mn} ', fg='bright_white', nl=False)
            if mn == metric_names[0]:
                click.secho(' (from ', nl=False)
                click.secho(metric_fne.__module__, fg='green', nl=False)
                click.secho(')')
            else:
                click.secho()

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
        else:
            missing.append(f'{metric_fne.__module__}/{metric_name}')

        params = get_parameter_dependencies(metric_name)
        if params:
            click.secho('\tParameters:', fg='blue')
            for param in params:
                click.secho(f'\t  {param:20} (default={param_defaults[param]})', fg='blue')

        click.secho('')

    if args.debug and missing:
        click.secho('Metrics missing docstring:', fg='red')
        for m in missing:
            click.secho(m, fg='red')
