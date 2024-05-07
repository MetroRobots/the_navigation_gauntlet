def get_values(pconfig, default_n=5):
    if isinstance(pconfig, list):
        return pconfig
    elif not isinstance(pconfig, dict):
        return [pconfig]

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
    for name, pconfig in sorted(parameters.items()):
        new_configs = []
        for value in get_values(pconfig, default_n):
            for config in configs:
                new_config = dict(config)
                new_config[name] = value
                new_configs.append(new_config)
        configs = new_configs
    return configs
