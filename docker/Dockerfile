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
ARG CARTOGRAPHER_SHA=877157a0d91788a7700221d87232d412cb3c1ef4
ARG CARTOGRAPHER_ROS_SHA=c138034db0c47fe0ea5a2abe516acae02190dbf5

# Create Cartographer workspace.
RUN mkdir -p /opt/cartographer_ws/src

# Fetch and pin Cartographer.
RUN git clone https://github.com/cartographer-project/cartographer.git \
        /opt/cartographer_ws/src/cartographer \
    && git -C /opt/cartographer_ws/src/cartographer \
        checkout ${CARTOGRAPHER_SHA}

# Fetch and pin Cartographer ROS.
RUN git clone https://github.com/cartographer-project/cartographer_ros.git \
        /opt/cartographer_ws/src/cartographer_ros \
    && git -C /opt/cartographer_ws/src/cartographer_ros \
        checkout ${CARTOGRAPHER_ROS_SHA}

# Abseil is installed separately using Cartographer's pinned installer.
RUN sed -i \
    's#<depend>libabsl-dev</depend>#<!-- <depend>libabsl-dev</depend> -->#' \
    /opt/cartographer_ws/src/cartographer/package.xml

# Install system / ROS dependencies required by Cartographer.
RUN apt-get update \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash \
        && rosdep update --include-eol-distros --rosdistro=noetic \
        && rosdep install \
            --from-paths /opt/cartographer_ws/src \
            --ignore-src \
            --rosdistro=noetic \
            -y" \
    && rm -rf /var/lib/apt/lists/*

# Install Cartographer's pinned Abseil version.
RUN cd /opt/cartographer_ws \
    && ./src/cartographer/scripts/install_abseil.sh

# Build and install Cartographer ROS.
RUN /bin/bash -c \
    "source /opt/ros/noetic/setup.bash && \
     cd /opt/cartographer_ws && \
     catkin_make_isolated --install --use-ninja"

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

# --------------------------------------------------
# P2-A inference: isolated Python, never replace ROS Python or activate globally
# Build context is the repository root; use docker/Dockerfile.dockerignore.
# --------------------------------------------------
COPY car_2026/src/robot_perception/docs/requirements-cpu.lock.txt /opt/robot-perception/requirements-cpu.lock.txt

RUN python3 -m pip install --disable-pip-version-check --no-cache-dir \
    --target /opt/robot-perception/bootstrap uv==0.8.22

USER ${USERNAME}

RUN export UV_PYTHON_INSTALL_DIR=/home/${USERNAME}/.local/share/uv/python \
           UV_CACHE_DIR=/home/${USERNAME}/.cache/uv \
    && /opt/robot-perception/bootstrap/bin/uv --native-tls venv \
        --python 3.10.18 /home/${USERNAME}/.venvs/robot-perception \
    && /opt/robot-perception/bootstrap/bin/uv --native-tls pip install \
        --python /home/${USERNAME}/.venvs/robot-perception/bin/python \
        --index-url https://download.pytorch.org/whl/cpu \
        --constraint /opt/robot-perception/requirements-cpu.lock.txt \
        torch==2.14.0+cpu torchvision==0.29.0+cpu \
    && /opt/robot-perception/bootstrap/bin/uv --native-tls pip install \
        --python /home/${USERNAME}/.venvs/robot-perception/bin/python \
        --requirement /opt/robot-perception/requirements-cpu.lock.txt \
    && /opt/robot-perception/bootstrap/bin/uv pip check \
        --python /home/${USERNAME}/.venvs/robot-perception/bin/python \
    && /opt/robot-perception/bootstrap/bin/uv cache clean

# Import checks only: do not load weights or import HyperLPR before cache preparation.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    PYTHONNOUSERSITE=1 YOLO_OFFLINE=true YOLO_AUTOINSTALL=false \
    /home/${USERNAME}/.venvs/robot-perception/bin/python -c \
    \"import rospy, sensor_msgs.msg, std_msgs.msg; \
    import ultralytics, torch, torchvision, numpy, cv2, PIL, onnxruntime, yaml; \
    import importlib.metadata; \
    assert importlib.metadata.version('hyperlpr3') == '0.1.3'; \
    assert torch.version.cuda is None; \
    print('Isolated CPU inference dependencies and ROS messages import OK')\""

CMD ["/bin/bash"]
