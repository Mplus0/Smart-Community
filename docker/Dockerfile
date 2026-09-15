FROM osrf/ros:noetic-desktop-full

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------
# Base development tools
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    wget \
    vim \
    nano \
    tmux \
    tree \
    htop \
    net-tools \
    iputils-ping \
    build-essential \
    cmake \
    python3-pip \
    python3-catkin-tools \
    python3-wstool \
    python3-rosdep \
    ninja-build \
    stow \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# ROS / Gazebo / Navigation / SLAM dependencies
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-robot-localization \
    ros-noetic-teb-local-planner \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-xacro \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-view \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# Cartographer / Cartographer ROS
# --------------------------------------------------
# Fixed commits validated with ROS Noetic / Ubuntu 20.04.
ARG CARTOGRAPHER_SHA=877157a0d91788a7700221d87232d412cb3c1ef4
ARG CARTOGRAPHER_ROS_SHA=c138034db0c47fe0ea5a2abe516acae02190dbf5

RUN set -eux; \
    mkdir -p /opt/cartographer_ws/src; \
    git clone https://github.com/cartographer-project/cartographer.git \
        /opt/cartographer_ws/src/cartographer; \
    git -C /opt/cartographer_ws/src/cartographer \
        checkout ${CARTOGRAPHER_SHA}; \
    git clone https://github.com/cartographer-project/cartographer_ros.git \
        /opt/cartographer_ws/src/cartographer_ros; \
    git -C /opt/cartographer_ws/src/cartographer_ros \
        checkout ${CARTOGRAPHER_ROS_SHA}; \
    sed -i \
        's#<depend>libabsl-dev</depend>#<!-- <depend>libabsl-dev</depend> -->#' \
        /opt/cartographer_ws/src/cartographer/package.xml; \
    rosdep update; \
    rosdep install \
        --from-paths /opt/cartographer_ws/src \
        --ignore-src \
        --rosdistro=noetic \
        -y; \
    cd /opt/cartographer_ws; \
    ./src/cartographer/scripts/install_abseil.sh; \
    . /opt/ros/noetic/setup.sh; \
    catkin_make_isolated --install --use-ninja

# --------------------------------------------------
# Create non-root developer user
# --------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --------------------------------------------------
# ROS environment
# --------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/cartographer_ws/install_isolated/setup.bash" >> /home/${USERNAME}/.bashrc

RUN mkdir -p /workspace/car_2026/src \
    && chown -R ${USERNAME}:${USERNAME} /workspace

WORKDIR /workspace/car_2026

USER ${USERNAME}

CMD ["/bin/bash"]
