.. _installation:


Installation
------------

Prerequisites
^^^^^^^^^^^^^

In order to install MOON, ``pip`` is required as well as an installation of ROS2 up until `Jazzy Jalisco <https://docs.ros.org/en/jazzy/index.html>`_, and the following Python packages:

- ``websocket_client``
- ``rospy_message_converter``
- ``pyyaml``
- ``reelay``

In order for Reelay to work, an installation of the `Boost <https://www.boost.org/>`_ C++ libraries is necessary.

Installation
^^^^^^^^^^^^

We need to clone the MOON repository, making sure the ROSMonitoring submodule is cloned too by using ``--recursive``.

.. code-block:: bash

    $ git clone --recursive https://github.com/convince-project/MOON.git
    
