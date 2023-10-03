import re

OPERATION_PATTERN = re.compile(r'^([\w_]+)(%|[<>=]=?)(.*)$')

OPPOSITE_OPERATION = {
    '<': '>=',
    '<=': '>',
    '>': '<=',
    '>=': '<',
}


class Dimension:
    """Helper class for analyzing one dimension of a set of metrics."""

    def __init__(self, full_name, allow_str=False):
        self.full_name = full_name
        self.allow_str = allow_str
        self.count = 0              # Count how many times this dimension was found
        self.alter_fne = None
        self.op = None
        self.operand = None

        if not self.full_name:
            self.name = ''
            return

        m = OPERATION_PATTERN.match(self.full_name)
        if m:
            self.name, self.op, param = m.groups()
            self.operand = eval(param)
            if self.op == '%':
                self.alter_fne = lambda d: d - d % self.operand
            else:
                self.alter_fne = lambda d: self.eval_op(d)
        else:
            self.name = self.full_name

    def eval_op(self, d, debug=False):
        try:
            return eval(f'{d} {self.op} {self.operand}')
        except Exception as e:
            if debug:
                print(e)
            return None

    def get_value(self, metric_d):
        if not self.full_name:
            return

        # Look in both metrics and parameters
        if self.name in metric_d:
            value = metric_d[self.name]
        elif self.name in metric_d.get('parameters', {}):
            value = metric_d['parameters'][self.name]
        else:
            value = None

        # Avoid plotting error strings
        if not self.allow_str and isinstance(value, str):
            value = None

        if self.alter_fne and value:
            value = self.alter_fne(value)

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
                return f'{self.name} ∈ [{value:.2f}, {(value + self.operand):.2f}]'
            else:
                return f'{self.name} = {value}'
        elif value:
            return f'{self.name} {self.op} {self.operand}'
        elif value is None:
            return f'{self.name} = None'
        else:
            return f'{self.name} {OPPOSITE_OPERATION[self.op]} {self.operand}'

    def __repr__(self):
        return self.name


def matches_any(metrics, dimensions):
    for dimension in dimensions:
        d_v = dimension.get_value(metrics)
        if d_v:
            return True
