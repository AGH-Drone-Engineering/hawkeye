FROM carbon225/ros-gazebo-ardu:humble-fortress

RUN apt-get update

ENV PROJECT_ROOT=/workspace

RUN echo source '${PROJECT_ROOT}/ros2_ws/install/setup.bash' >> /root/.bashrc
ENV PATH=${PROJECT_ROOT}/scripts:$PATH
ENV IGN_GAZEBO_RESOURCE_PATH=${PROJECT_ROOT}/models:${PROJECT_ROOT}/worlds:$IGN_GAZEBO_RESOURCE_PATH

WORKDIR ${PROJECT_ROOT}/ros2_ws
