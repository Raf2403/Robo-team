cmake_minimum_required(VERSION 3.0.2)
project(mastering_ros_robot_description_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rviz
  tf
  urdf
  xacro
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    roscpp 
    rviz 
    tf 
    urdf 
    xacro 
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/a_star_planner.py scripts/robot_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
