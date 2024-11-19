# MOON

MOON (MOnitoring ONline) is a runtime monitoring framework developed for CONVINCE on top of the ROSMonitoring tool, providing monitor generation for properties and models. Currently, only monitor generation for properties is implememented on top of [ROSMonitoring](https://github.com/autonomy-and-verification-uol/ROSMonitoring/tree/ros2), working for ROS2 topics and services.

MOON will notify violations of properties and that other tools can be invoked to amend plans or models and adapt the control architecture to new and unforeseen situations.

MOON uses PastMTL as property specification language, relying on Reelay for their verification.

In perspective, the tool will include also monitoring for models, i.e., the capability of ensuring that the concrete execution of some elements of the control architecture or the environment correspond to the abstract model utilized at design-time.

## Installation and usage

### Prerequisites

`pip` and an installation of ROS2 up until Iron Irwini (recommended) is required, and so are the following Python packages:
- `websocket_client`
- `rospy_message_converter`
- `pyyaml`
- `reelay`

In order for Reelay to work, an installation of the `boost` library is necessary.

### Installation

We need to clone the ros2 branch of the ROSMonitoring repository.
```sh
$ git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2
```

### Usage

In order to use the tool, we need to define a monitor configuration, such as the following:

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
      services: # list of services the monitor intercepts
        - name: my_service # name of the service
          type: std_msgs.msg.String # type of the service
          action: log
```
 Then, we need to generate the corresponding monitor, by invoking the `generator` command.

 ```bash
 $ /path/to/ROSMonitoring/generator/ros2_devel/generator --config-file /path/to/monitor_config.yaml
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

Then, we need to run the oracle by specifying the property and whether the time events are evenly spaced out or not, by setting either the `--dense` or `--discrete` flag.

```bash
$ /path/to/ROSMonitoring/oracle/TLOracle/oracle.py --online --property /path/to/prop --port 8080 --dense
```

We can now run the monitor, with
```bash
$ cd /path/to/monitor_ws
$ . install/setup.bash
$ ros2 launch src/monitor/launch/monitor.launch
```

Now the monitor will be running, and when the monitored topic/service is running, it will send messages to the oracle, which will provide an evaluation of the property.

## Running example

An example of a running monitoring execution can be found within the `docker` directory of the repository. It can be run by following the instructions within.
