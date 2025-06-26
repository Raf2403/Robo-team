# Mobile Robot Simulation in ROS Noetic and Gazebo

<img src="imgs/priview.jpg" alt="Симуляция проекта" width="600">

This project demonstrates the process of creating simulation models for various types of robots and manipulators, with a detailed explanation of building a URDF model of a four-wheeled mobile robot using ROS Noetic and Gazebo. It includes the development and design of the model, as well as integration with simulation environments and programming of robot behavior.

## See article
- [Russian article](/docs/Ru.md)

## Project Overview

This work aims to assist beginner developers in mastering the process of creating and testing simulation models of robots. It provides a foundation for further work and research in the field of robotics by offering practical knowledge and skills for developing complex robotic systems.

### Key Features

- **URDF Model**: Detailed description and creation of a URDF model for the mobile robot.
- **Docker Container**: Development of a Docker container for ease of deployment and configuration.
- **Configuration Files**: Setup and explanation of configuration files for operating within Gazebo.
- **Custom Gazebo World**: Formation of a custom simulation environment in Gazebo.
- **PID Controllers**: Setup and tuning of PID controllers for optimal motion control.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

#### Local Setup
- ROS Noetic
- Gazebo (compatible with ROS Noetic)

#### Docker Setup
- Docker
- Docker Compose

### Installation

#### Local Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/AntonSHBK/mobile_robot_base_ros1.git
   ```

2. **Build the project using catkin_make:**
   ```bash
   cd mobile_robot_base_ros1
   catkin_make -DCATKIN_WHITELIST_PACKAGES="mobile_robot_base_ros1"
   ```

#### Docker Installation

1. **Build the Docker image:**
   ```bash
   cd mobile_robot_base_ros1/docker
   docker-compose build
   ```
2. **Run the Docker image:**
   ```bash
   docker-compose up
   ```
### Running the Simulations

- **Launch the simulation environment in Gazebo and RViz:**
  ```bash
  source devel/setup.bash
  roslaunch mobile_robot_base_ros1 robot_base.launch
  ```

## PID Controller Configuration

To adjust the PID settings, modify the `pid_params.yaml` under the `config` directory.

## Customizing the Gazebo World

The custom world can be modified by editing the `empty_world.world` file located in the `world` directory to change simulation parameters or environment layouts.


## Authors

- **Anton Pisarenko** - *Initial work* - [AntonSHBK](https://github.com/AntonSHBK)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Citation

```latex
@misc{pisarenko2023mobile,
  author = {Pisarenko, Anton},
  title = {Mobile Robot Simulation in {ROS} Noetic and Gazebo},
  year = {2023},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/AntonSHBK/mobile_robot_base_ros1}}
}
```