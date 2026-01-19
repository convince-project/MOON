# MOON

MOON (MOnitoring ONline) is a runtime monitoring framework developed for CONVINCE on top of the [ROSMonitoring](https://github.com/autonomy-and-verification-uol/ROSMonitoring/tree/ros2) tool, providing monitor generation for properties and models.

MOON will notify violations of properties and that other tools can be invoked to amend plans or models and adapt the control architecture to new and unforeseen situations.MOON uses PastMTL as property specification language, relying on Reelay for their verification.

## Installation and usage

### Prerequisites

`pip` and an installation of ROS2 up until [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html) are required.

In order for Reelay to work, an installation of the [`boost`](https://www.boost.org/) library is necessary.

On Ubuntu 24.04, this can be done with
``` bash
$ sudo apt install libboost-all-dev libcairo2-dev
```

### Clone the MOON repo

We need to clone the MOON repository with
```sh
$ git clone https://github.com/convince-project/MOON.git
```

### Installation
MOON can be installed as a Python package with

```bash
$ pip install MOON/
```

### Usage

In order to use the tool, we need to define a monitor configuration, like in the following:

```yaml
path: /path/to/monitor/workspace/src/ # path to the ros workspace of the monitor package

monitors: # list of generated monitors
  - monitor:
      id: my_monitor # monitor id
      log: ./log.txt # file where the monitor will log the observed events
      silent: False # the monitor prints info during its execution
      oracle: # the oracle running and ready to check the specification
        port: 8080 # port where the oracle is listening
        url: 127.0.0.1 # url where itthe oracle is listening
        action: nothing # action performed by the oracle
      topics: # list of topics this monitor is going to intercept
        - name: my_topic # name of the topic
          type: std_msgs.msg.String # type of the topic
          action: log
          qos_reliability: reliable  # topic reliability (reliable/best_effort)
      services: # list of services the monitor intercepts
        - name: my_service # name of the service
          type: std_msgs.msg.String # type of the service
          action: log
      actions: # list of actions the monitor intercepts
        - name: my_action # name of the action
          type: custom_action_interfaces.action.MyAction # type of the action
          action: log
```


Then, we need to generate the corresponding monitor, by invoking the `moon_generator` command.

 ```bash
 $ moon_generator --config-file /path/to/monitor_config.yaml
 ```

 Now we need to build the newly created ROS package, so we run
 ```bash
 $ cd /path/to/monitor/workspace
 $ colcon build
 ```

Next, we need to define a property to be verified on the monitored topics and/or services, such as

```python
import oracle

# property to verify
PROPERTY = "historically{p}"

# declaration of predicates used in the property (initialization at time 0)
predicates = dict(
    time = 0,
    p = True,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
# the behavior of the function must be defined by the user depending on the property and topic/service message
def abstract_message(message):
    predicates['time'] = message['time']
    predicates['p'] = message['p']
    return predicates
```
The PastMTL property needs to be defined as a string assigned to the `PROPERTY` variable, following the [Reelay syntax](https://doganulus.github.io/reelay/rye/).

Then, we need to run the oracle by specifying the property and whether the time events are evenly spaced out or not, by setting either the `--dense` or `--discrete` flag.

```bash
$ moon_oracle --online --property /path/to/prop --port 8080 --dense
```

We can now run the monitor, with
```bash
$ cd /path/to/monitor_ws
$ source install/setup.bash
$ ros2 run monitor my_monitor
```

In case any action monitors were defined, we need to also start the corresponding node in the same workspace
```bash
$ ros2 run my_action_monitor my_action_monitor
```

Now the monitor will be running, and when the monitored channel is running, it will send messages to the oracle, which will provide an evaluation of the property.

## Running example

An example of a running monitoring execution can be found within the `examples/tutorial` directory of the repository. At least three terminals are needed: one for the monitor, one for the oracle and one to execute the ROS topic to be monitored.

In the first terminal, execute the following commands inside the MOON directory, after sourcing the ROS distribution setup file
```bash
$ moon_generator ./examples/tutorial/monitor_example.yaml
$ cd ./monitor_ws
$ colcon build
$ source install/setup.bash
```
Then, in another terminal run
```bash
$ moon_oracle --online --dense --port 8080 --property ./examples/tutorial/property_example
```
While the oracle is running, we then run the monitor in the previous terminal
```bash
$ ros2 run monitor example_monitor
```

Now the monitor is connected to the oracle, waiting for messages to be published.
In order to see it in action, we need to run the chatter topic publisher from the ROS demo package with
```bash
$ ros2 run demo_nodes_cpp talker
```
The monitor now should be publishing the verdict on the property on each message it intercepts.
