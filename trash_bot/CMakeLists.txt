cmake_minimum_required(VERSION 3.8)
project(trash_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts with proper names (stripping .py extension)
install(PROGRAMS trash_bot/gesture_control.py DESTINATION lib/${PROJECT_NAME} RENAME gesture_control)
install(PROGRAMS trash_bot/mission_controller.py DESTINATION lib/${PROJECT_NAME} RENAME mission_controller)
install(PROGRAMS trash_bot/serial_bridge.py DESTINATION lib/${PROJECT_NAME} RENAME serial_bridge)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
