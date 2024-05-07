# ![The Navigation Gauntlet](NavigationGauntlet.png)

An open framework for testing navigation algorithms using different robots and different simulators.

## Running Trials: The `gauntlet_runner` package
At the core of the tests is running a single trial, which consists of simulating a robot performing one [`NavigateToPose`](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action.

There are four core variables:
 * **Simulator**
 * **Robot**
 * **Nav Config**
 * **Trial Config**

The simulator package should have a simulator_bringup.launch.py file and have three launch arguments:
 * `world` - The path to the [YamlWorld](YamlWorld.md) world configuration.
 * `robot` - The name of a robot to launch
 * `gui` - Boolean to determine whether to display a GUI window for the simulator

The simulator, robot and nav config packages should have `config/nav_gauntlet.yaml` defined. These files may contain a list of topics to record.
