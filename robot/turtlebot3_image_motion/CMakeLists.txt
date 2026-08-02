cmake_minimum_required(VERSION 3.8)
project(turtlebot3_image_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Motion.srv"
)

# Install Python executables
install(PROGRAMS
  turtlebot3_image_motion/image_publisher.py
  turtlebot3_image_motion/web_controller.py
  turtlebot3_image_motion/motion_server.py
  turtlebot3_image_motion/mcp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package for imports
install(DIRECTORY turtlebot3_image_motion/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
