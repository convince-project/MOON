```
docker compose run --name mooncont -it base bash
docker exec -it mooncont bash

cd /MOON/src/ROSMonitoring/oracle/TLOracle/
oracle.py --online --property prop --port 8080 --dense

python3 generator --config_file ../roscon_res/conf.yaml
cd /MOON/monitor_ws; colcon build
ros2 run monitor pick_monitor

cd /MOON/roscon_res/sim_ws; colcon build
ros2 run simexec exec
```