.. _usage:


Usage
-----

In order to use the tool, we need to define a monitor configuration, such as the following:

.. code-block:: yaml

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
        actions: # list of actions the monitor intercepts
            - name: my_action # name of the action
            type: custom_action_interfaces.action.MyAction # type of the action
            action: log

Then, we need to generate the corresponding monitor, by invoking the ``generator`` command.

.. code-block:: bash

    $ /path/to/MOON/src/generator --config-file /path/to/monitor_config.yaml

Now we need to build the newly created ROS package, so we run

.. code-block:: bash
    $ cd /path/to/monitor/workspace
    $ colcon build

Next, we need to define a property to be verified on the monitored topics and/or services, such as

.. code-block:: python
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

Then, we need to run the oracle by specifying the property and whether the time events are evenly spaced out or not, by setting either the ``--dense`` or ``--discrete`` flag.

.. code-block:: bash
    $ /path/to/MOON/src/ROSMonitoring/oracle/TLOracle/oracle.py --online --property /path/to/prop --port 8080 --dense

.. code-block:: bash
    $ cd /path/to/monitor_ws
    $ . install/setup.bash
    $ ros2 launch src/monitor/launch/monitor.launch

Now the monitor will be running, and when the monitored topic/service is running, it will send messages to the oracle, which will provide an evaluation of the property.
