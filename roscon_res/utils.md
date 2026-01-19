```
docker compose run --name mooncont -it base bash
docker exec -it mooncont bash

cd /MOON/src/ROSMonitoring/oracle/TLOracle/
cp /MOON/roscon_res/prop1.py /MOON/src/ROSMonitoring/oracle/TLOracle
oracle.py --online --property prop1 --port 8080 --dense

python3 generator --config_file ../roscon_res/conf.yaml
cd /MOON/monitor_ws; colcon build
source install/setup.bash
ros2 run monitor pick_monitor

source /opt/ros/jazzy/setup.bash
cd /MOON/roscon_res/sim_ws; colcon build
source install/setup.bash
ros2 run simexec exec
```