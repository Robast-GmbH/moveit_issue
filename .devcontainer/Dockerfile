FROM moveit/moveit2:iron-release

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USER/.bashrc

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \	 
    gnupg \	  
    git \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    cmake \
    gdb \
    pylint \ 
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libboost-all-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"
    
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

#install to use the nvidia gpu in the docker
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

# Creating a new separate workspace to install moveit_task_constructor because its not available as package yet (as at 13.09.23)
RUN mkdir -p workspace_moveit2/src && cd workspace_moveit2/src && git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2 --depth 1 \
  && apt update \
  && apt upgrade -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && rosdep update \
  && rosdep install --from-paths /workspace_moveit2/src --ignore-src -r -y \
  && colcon build --parallel-workers 3 --build-base /workspace_moveit2/build --install-base /workspace_moveit2/install \
  && echo "if [ -f /workspace_moveit2/install/setup.bash ]; then source /workspace_moveit2/install/setup.bash; fi" >> /home/$USER/.bashrc

# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# rmw_cyclonedds_cpp #rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/workspace/dds_configs/cyclone_DDS_config.xml
ENV IGN_FILE_PATH=/workspace/src/simulation/tiplu_world/models:${GAZEBO_MODEL_PATH}
ENV is_simulation=True
ENV config_directory=config_simulation
ENV robot_name=rb_theron
ENV init_x=0.0
ENV init_y=0.0
ENV init_z=0.0
ENV init_yaw=0.0
ENV robot=rb_theron
ENV prefix=