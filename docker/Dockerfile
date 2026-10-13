FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo package repository (provides python3-sdformat14)
RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release gnupg ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      python3-sdformat14 \
      ros-jazzy-ros-gz \
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-xacro \
      python3-colcon-common-extensions \
      python3-pip \
      git \
      sudo \
      mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# VRX is baked into the image so every container has it
RUN mkdir -p /opt/vrx_ws/src \
 && git clone --depth 1 https://github.com/osrf/vrx.git /opt/vrx_ws/src/vrx \
 && cd /opt/vrx_ws \
 && apt-get update \
 && rosdep update --rosdistro jazzy \
 && (rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true) \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --merge-install \
 && rm -rf /var/lib/apt/lists/*

# Source ROS 2, VRX and (if built) our workspace in every shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
 && echo 'source /opt/vrx_ws/install/setup.bash' >> /etc/bash.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /etc/bash.bashrc

# Same UID as the host user (1000) so bind-mounted files keep sane ownership
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && mkdir -p /ws /home/ubuntu/.gz \
 && chown -R ubuntu:ubuntu /ws /home/ubuntu/.gz /opt/vrx_ws

USER ubuntu
WORKDIR /ws
CMD ["sleep", "infinity"]
