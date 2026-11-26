FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive

# sim, bridges, recording, dev tools, terminal tooling, rendering fallback
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rosbag2-storage-mcap \
        ros-dev-tools \
        tmux tmuxinator \
        mesa-utils libegl1 libgl1 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Environment: UTF-8 text, and GPU graphics/EGL libraries (only used with --gpus all)
ENV LANG=C.UTF-8 \
    NVIDIA_DRIVER_CAPABILITIES=all

# Source ROS and the colcon workspace in every shell (also `docker exec` ones)
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /etc/bash.bashrc

COPY docker/.tmux.conf /root/.tmux.conf
# Where gz would look for resources: first in models/, then in worlds/
ENV GZ_SIM_RESOURCE_PATH=/workspace/simulation/models:/workspace/simulation/worlds
WORKDIR /workspace