cmake_minimum_required(VERSION 3.5)
project(virtual_data_publisher)

# 设置C++标准（如果未指定则使用 C++14）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# （可选）添加include目录，如果你有自定义头文件放在 include 目录下
include_directories(include)

# 添加可执行文件，这里将编译 src/virtual_data_publisher.cpp 文件
add_executable(virtual_data_publisher
  src/virtual_data_publisher.cpp
)

# 链接所需的依赖库
ament_target_dependencies(virtual_data_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# 安装可执行文件到ROS2包的 lib 目录下
install(TARGETS
  virtual_data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 声明该包
ament_package()
