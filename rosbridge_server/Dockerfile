FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rosbridge-server ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
