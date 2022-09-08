FROM carbon225/ros-gazebo-ardu:humble-fortress

RUN apt-get update

ARG PROJECT_ROOT=/workspace
RUN echo source ${PROJECT_ROOT}/ros2_ws/install/setup.bash >> /root/.bashrc
ENV PATH=${PATH}:${PROJECT_ROOT}/scripts
ENV IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:${PROJECT_ROOT}/simulation/models:${PROJECT_ROOT}/simulation/worlds

RUN mkdir /wdir
WORKDIR /wdir
