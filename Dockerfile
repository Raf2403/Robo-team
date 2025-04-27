# Использование базового образа ROS Noetic
FROM osrf/ros:noetic-desktop

# Установка переменных среды
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Создание и установка рабочей директории
WORKDIR /workspace

# Установка системных утилит
RUN apt-get update && \
    apt-get install -y \
    curl \
    wget \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Установка Gazebo 11


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
COPY ./catkin_ws/src /workspace/src

# Финализация
RUN echo "All done!"