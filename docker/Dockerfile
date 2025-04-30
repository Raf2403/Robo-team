FROM osrf/ros:noetic-desktop-full

# Установка переменных среды
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-wstool \
    python3-vcstools \
    python3-osrf-pycommon \
    vim \
    tmux \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Установка ключевых ROS-пакетов
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt* \
    ros-noetic-common-msgs \
    ros-noetic-tf2-tools \
    ros-noetic-rosbash \
    ros-noetic-roslaunch \
    ros-noetic-actionlib \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-nodelet \
    ros-noetic-pluginlib \
    ros-noetic-urdfdom-py \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-xacro \
    ros-noetic-kdl-parser \
    ros-noetic-tf-conversions \
    ros-noetic-control-toolbox \
    ros-noetic-realtime-tools \
    ros-noetic-ros-controllers \
    ros-noetic-navigation \
    liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

# Установка Python-зависимостей
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    scipy \
    matplotlib \
    pandas \
    pyyaml \
    rospkg \
    catkin_pkg \
    empy \
    defusedxml

#gedit and VScode
RUN apt-get update && apt-get install -y gedit && \
    rm -rf /var/lib/apt/lists/*
    
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/ && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list && \
    apt-get update && apt-get install -y code && \
    rm -rf /var/lib/apt/lists/*

# Настройка окружения
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "alias cs='cd /catkin_ws && source devel/setup.bash'" >> ~/.bashrc

# Фиксируем версии Gazebo (опционально)
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# Финализация
RUN echo "ROS Noetic environment ready!"