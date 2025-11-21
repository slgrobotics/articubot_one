cmake_minimum_required(VERSION 3.8)
project(articubot_one)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (articubot_one/) - "helpers" includes
ament_python_install_package(${PROJECT_NAME})

# Install all directories recursively
install(
  DIRECTORY assets config description launch robots
  DESTINATION share/${PROJECT_NAME}
)

# Provide resource index for python package - "helpers" includes
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/gps_waypoint_follower.py
  scripts/xy_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
