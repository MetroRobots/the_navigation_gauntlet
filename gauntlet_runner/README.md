## Specifying Exploration Space for the Navigation Gauntlet

When you run trials with `ros2 run gauntlet_runner run_trials`, the [exploration.py](gauntlet_runner/exploration.py) module provides logic for systematically exploring the parameter space.

There are XX different ways to specify a parameter.

### Explicit Values
You can specify the `values` for a parameter as a list of values. For example,

```
parameters:
- name: nav/pkg
  values:
  - nav_pkg1
  - nav_pkg2
- name: sim/spawn_frequency
  values: [2.0, 5.0, 10.0]
```

Note that this is currently the only way to specify different values for string parameters.

### Explicit Boolean Values
As a minor shortcut, you can also just specify `type: bool` to get the equivalent of `values: [False, True]`

### Unbounded Range
If you don't specify `type` or `values`, then we will explore some number of numeric values, based on two parameters:
 * `min` (default: 0) - the minimum value for the parameter
 * `n` (default: 5) - the number of values to explore

If you specify `max`, it will be a bounded range (see below).

Hence, if you just specify
```
parameters:
- name: trial/trial_no
```
it will default to explore 5 values of `trial_no`, starting at 0, i.e. `0, 1, 2, 3, 4`.
```
parameters:
- name: trial/trial_no
  n: 8
  min: 10
```
This will generate 8 values of `trial_no` starting at 10, i.e. `10, 11, 12, 13, 14, 15, 16, 17`



### Bounded Range
You can also specify `max` to get a bounded range. For example,
```
parameters:
- name: sim/spawn_frequency
  max: 1.0
```
This will generate 5 values of `spawn_frequency` starting at 0, but now all the values are between 0 and 1, i.e. `[0.0, 0.25, 0.5, 0.75, 1.0]`

Increasing the `n` to 6 will result in the same endpoints, but disparate internal values, i.e. `[0.0, 0.2, 0.4, 0.6, 0.8, 1.0]`.

A wiser move might be to increase `n` to 9, thereby getting some overlap with the first set of values, i.e. `[0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]`


## Formatting Values
The one additional parameter you can specify with the parameters is `format` which does not change how the parameters are explored, but how they are formatted when creating folder paths. By default, `str(value)` is called, but if you specify `format`, it is equivalent to running `format.format(value)`.

Without a format specified,
```
- name: trial/trial_no
```
will produce folders named
`trial_no_0`, `trial_no_1`, `trial_no_2`, `trial_no_3`, `trial_no_4`,

For integer parameters, you can specify leading zeroes, e.g.
```
- name: trial/trial_no
  format: '{:03d}'
```
which will produce `trial_no_000`, `trial_no_001`, `trial_no_002`, `trial_no_003`, `trial_no_004`,

It is also handy for specifying floating point precision, e.g.

```
- name: sim/spawn_frequency
  values: [2.0, 5.0, 10.0]
  format: '{:05.2f}'
```
will produce folders named
`spawn_frequency_02.00`, `spawn_frequency_05.00`, `spawn_frequency_10.00` instead of `spawn_frequency_2.0`, `spawn_frequency_5.0`, `spawn_frequency_10.0`.
