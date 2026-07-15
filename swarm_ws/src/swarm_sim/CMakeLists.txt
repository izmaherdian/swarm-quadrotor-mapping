cmake_minimum_required(VERSION 3.8)
project(swarm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install folder simulasi (agar dikenali oleh ROS 2 saat di-build)
install(DIRECTORY
  launch
  models
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
