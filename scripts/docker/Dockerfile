FROM ros:foxy

RUN echo "source /opt/ros/foxy/setup.sh" >> ~/.bashrc

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

RUN apt-get update -y
RUN apt-get install -y libasio-dev

RUN git clone https://github.com/westonrobot/ugv_sdk.git
RUN git clone https://github.com/agilexrobotics/tracer_ros2.git

WORKDIR /ros2_ws
RUN . /opt/ros/foxy/setup.sh && colcon build

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "install/setup.sh"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "tracer_base", "tracer_base.launch.py"]