cmake_minimum_required(VERSION 3.5)
project(lab1_pkg)

# Find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/talker.py
  scripts/relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
