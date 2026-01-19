.. _tutorial:


Tutorial
--------

An example of a running monitoring execution can be found within the ``examples/tutorial`` directory of the repository. At least three terminals are needed: one for the monitor, one for the oracle and one to execute the ROS topic to be monitored.

In the first terminal, execute the following commands inside the MOON directory, after sourcing the ROS distribution setup file

.. code-block:: bash

    $ moon_generator ./examples/tutorial/monitor_example.yaml
    $ cd ./monitor_ws
    $ colcon build
    $ source install/setup.bash

Then, in another terminal run

.. code-block:: bash

    $ moon_oracle --online --dense --port 8080 --property ./examples/tutorial/property_example

While the oracle is running, we then run the monitor in the previous terminal

.. code-block:: bash

    $ ros2 run monitor example_monitor

Now the monitor is connected to the oracle, waiting for messages to be published. In order to see it in action, we need to run the chatter topic publisher from the ROS demo package with

.. code-block:: bash

    $ ros2 run demo_nodes_cpp talker

The monitor now should be publishing the verdict on the property on each message it intercepts.
