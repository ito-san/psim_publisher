cmake_minimum_required(VERSION 3.5)
project(psim_publisher)

## Compile as C++14, supported in ROS Melodic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(psim_publisher
  src/psim_publisher_node.cpp
  src/psim_publisher.cpp
)

install(TARGETS psim_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
