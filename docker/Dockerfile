# Использование базового образа ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Установка переменных среды
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Создание и установка рабочей директории
WORKDIR /catkin_ws

# Установка системных утилит
RUN apt-get update && \
    apt-get install -y \
    curl \
    wget \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Gazebo 11
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get install -y libgazebo11-dev 
# RUN sudo apt-get install -y ros-noetic-gazebo-ros
# RUN sudo apt-get install -y gazebo11

# RUN curl -sSL http://get.gazebosim.org | sh
RUN apt-get update && \
    apt-get install -y \
#     gazebo11 \
#     libgazebo11-dev 
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Установка пакетов ROS
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-control-toolbox \
    ros-noetic-realtime-tools \
    ros-noetic-ros-controllers \
    ros-noetic-xacro \
    python3-wstool \
    ros-noetic-tf-conversions \
    ros-noetic-kdl-parser \
    liburdfdom-tools && \
    rm -rf /var/lib/apt/lists/*

# Установка инструментов разработки
RUN apt-get update && \
    apt-get install -y \
    vim \
    git \
    tmux \
    tree && \
    rm -rf /var/lib/apt/lists/*

# Обновление pipы
RUN pip install --upgrade pip

# Настройка окружения
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "alias dros='source devel/setup.bash'" >> ~/.bashrc

# Копирование проекта (универсальный путь)
# COPY . /catkin_ws/src
#COPY ./catkin_ws /project/catkin_ws

# Финализация
RUN echo "All done!"