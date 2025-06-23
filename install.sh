#!/bin/sh

# ROSMonitoring installation script for Ubuntu 22.04
git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2

pip install websocket_client rospy_message_converter pyyaml jedi prompt_toolkit reelay
sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0 python3-colcon-common-extensions

cp ./generator ./ROSMonitoring/generator/ros2_devel/generator
#cp ./generator.py ./ROSMonitoring/generator/ros2_devel/generator.py
