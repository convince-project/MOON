.. _installation:


Installation
------------

Prerequisites
^^^^^^^^^^^^^

In order to install MOON, ``pip`` is required as well as an installation of ROS2 up until `Jazzy Jalisco <https://docs.ros.org/en/jazzy/index.html>`_

In order for Reelay to work, an installation of the `Boost <https://www.boost.org/>`_ C++ libraries is necessary.
On Ubuntu 24.04, this can be done with:

.. code-block:: bash

    $ sudo apt install libboost-all-dev libcairo2-dev

Installation
^^^^^^^^^^^^

We need to clone the MOON repository and install the MOON package.

.. code-block:: bash

    $ git clone https://github.com/convince-project/MOON.git
    $ pip install MOON/
    
