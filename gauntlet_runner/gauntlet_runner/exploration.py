def get_values(pconfig, default_n=5):
    if 'values' in pconfig:
        return pconfig['values']
    elif 'value' in pconfig:
        return [pconfig['value']]

    if pconfig.get('type') == 'bool':
        return [False, True]

    n = pconfig.get('n', default_n)
    min_value = pconfig.get('min', 0)
    if 'max' not in pconfig:
        return [min_value + i for i in range(n)]
    else:
        span = pconfig['max'] - min_value
        return [min_value + i * span / (n - 1) for i in range(n)]


def explore_parameter_space(parameters, default_n=5):
    configs = [{}]
    for pconfig in parameters:
        new_configs = []
        name = pconfig['name']
        for value in get_values(pconfig, default_n):
            for config in configs:
                new_config = dict(config)
                new_config[name] = value
                new_configs.append(new_config)
        configs = new_configs
    return configs


def format_value(pconfig, value, include_name=True):
    if 'format' in pconfig:
        fv = pconfig['format'].format(value)
    else:
        fv = str(value)
    if include_name:
        return pconfig['name'].partition('/')[2] + '_' + fv
    else:
        return fv
