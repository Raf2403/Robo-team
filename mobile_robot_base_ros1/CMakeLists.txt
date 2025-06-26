# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0.2)

# Set the project name
project(mobile_robot_base_ros1)

# Find the catkin packages required for the project
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    sensor_msgs
    cv_bridge
    image_transport
)

# Additional search for catkin components and OpenCV libraries
find_package(catkin REQUIRED COMPONENTS
    ${PKG_DEPS}
    ${OpenCV_INCLUDE_DIRS}
)

# Find the required version of the OpenCV package
find_package(OpenCV 4.2.0 REQUIRED)

# Define the catkin package
catkin_package()

# Add directories with header files
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(
    PROGRAMS
    scripts/camera_view.py
    scripts/control_gui.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add an executable file from the specified source file
add_executable(simple_motor_controller src/simple_controller.cpp)

# Link the executable file with libraries
target_link_libraries(
    simple_motor_controller 
    ${catkin_LIBRARIES}
)