cmake_minimum_required(VERSION 3.15)
project(articubot_one)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (articubot_one/) - "helpers" includes
ament_python_install_package(${PROJECT_NAME})

# Install all directories recursively
install(
  DIRECTORY assets config description docs launch robots
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
  scripts/run_battery_state_compat.py
  scripts/gps_waypoint_follower.py
  scripts/xy_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(battery_state_compat_test test/test_battery_state_compat.py)
  ament_add_pytest_test(map_orientation_launch_test test/test_map_orientation_launch.py)
  ament_add_pytest_test(wifi_survey_v2_launch_test test/test_wifi_survey_v2_launch.py)
endif()

ament_package()
