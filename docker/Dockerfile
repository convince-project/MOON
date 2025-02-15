ARG base_img=ste93/convince:qt_6.6.3_ubuntu22.04_iron_stable
FROM $base_img
LABEL maintainer="Stefano Bernagozzi"
USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt upgrade -y
RUN apt-get install -y libczmq-dev ros-iron-zmqpp-vendor-dbgsym ros-iron-zmqpp-vendor libczmq-dev libczmq4 curl 

USER user1

RUN pip install reelay websocket-client jedi prompt_toolkit

RUN /bin/bash -c "git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build"

#add fakeNavigation device
WORKDIR ${robotology_install_folder}
RUN cd yarp && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DYARP_COMPILE_libYARP_math=ON \
    -DYARP_COMPILE_GUIS=ON \
    -DYARP_COMPILE_DEVICE_PLUGINS=ON \
    -DENABLE_yarpcar_mjpeg=ON \
    -DENABLE_yarpcar_segmentationimage=ON \
    -DENABLE_yarpcar_portmonitor=ON \
    -DENABLE_yarpmod_portaudio=ON \
    -DENABLE_yarpmod_portaudioPlayer=ON \
    -DENABLE_yarpmod_portaudioRecorder=ON \
    -DENABLE_yarpmod_fakeAnalogSensor=ON \
    -DENABLE_yarpmod_fakeBattery=ON  \
    -DENABLE_yarpmod_fakeDepthCamera=ON \
    -DENABLE_yarpmod_fakeIMU=ON  \
    -DENABLE_yarpmod_fakeLaser=ON  \
    -DENABLE_yarpmod_fakeLocalizer=ON \
    -DENABLE_yarpmod_fakeMicrophone=ON  \
    -DENABLE_yarpmod_fakeMotionControl=ON  \
    -DENABLE_yarpmod_fakeNavigation=ON  \
    -DENABLE_yarpmod_fakeSpeaker=ON  \
    -DENABLE_yarpmod_fakebot=ON \
    -DENABLE_yarpmod_laserFromDepth=ON \
    -DENABLE_yarpmod_laserFromExternalPort=ON \
    -DENABLE_yarpmod_laserFromDepth=ON \
    -DENABLE_yarpmod_laserFromPointCloud=ON \
    -DENABLE_yarpmod_laserFromRosTopic=ON \
    -DENABLE_yarppm_soundfilter_resample=ON \
    -DENABLE_yarppm_bottle_compression_zlib=ON \
    -DENABLE_yarppm_depthimage_compression_zlib=ON \
    -DENABLE_yarppm_image_compression_ffmpeg=ON \
    -DENABLE_yarppm_depthimage_to_mono=ON \
    -DENABLE_yarppm_depthimage_to_rgb=ON && \
    make -j11


RUN cd yarp-devices-ros2/ros2_interfaces_ws && /bin/bash -c "source /opt/ros/iron/setup.bash; colcon build"
RUN cd yarp-devices-ros2 && cd build && /bin/bash -c "source /opt/ros/iron/setup.bash; source ${robotology_install_folder}/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash; cmake .. -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs=ON -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs=ON; make -j11"
    
WORKDIR /home/user1 

ENV Qt6_DIR=/home/user1/Qt6.6.3/lib/cmake

RUN /bin/bash -c "cd /home/user1 && rm -rf ROSMonitoring"

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/ste93/ROSMonitoring.git -b ros2"

# installing as2fm
RUN /bin/bash -c "python3 -m pip install --upgrade pip && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && \
    git clone https://github.com/convince-project/AS2FM.git && python3 -m pip install AS2FM/"

# install SCAN
USER user1
RUN /bin/bash -c "curl https://sh.rustup.rs -sSf | sh -s -- -y"

# curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -y| sh "

RUN /bin/bash -c "source /home/user1/.cargo/env && cargo install --git https://github.com/convince-project/scan"


RUN /bin/bash -c "echo 'updated on 202411141542' && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && \
    git clone https://github.com/convince-project/MBVM-SIMPAR2025.git -b  monitoring_prop3 && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1/MBVM-SIMPAR2025/code && \
    colcon build --packages-select notify_user_interfaces_dummy && \
    source /home/user1/MBVM-SIMPAR2025/code/install/setup.bash && \
    colcon build --packages-ignore BT face_expressions_wrapper bt_executable_reload"


RUN echo "source /home/user1/MBVM-SIMPAR2025/code/install/setup.bash" >> ~/.bashrc
RUN echo "export PATH=\$PATH:/home/user1/Groot/build" >> ~/.bashrc

RUN /bin/bash -c " echo 'updated on 202411171546' && \
    cd /home/user1/MBVM-SIMPAR2025 && git pull origin monitoring_prop3 && \
    cd /home/user1/ROSMonitoring/generator/ros2_devel && \
    source /opt/ros/iron/setup.bash && \
    python3 generator --config_file /home/user1/MBVM-SIMPAR2025/monitoring/monitor_prop3.yaml && \
    cd /home/user1/MBVM-SIMPAR2025/monitor_ws/src/ && \
    colcon build"

ENV PATH=${PATH}:/home/user1/.local/bin

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/src/robot/yarp/build/lib/
