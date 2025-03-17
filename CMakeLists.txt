cmake_minimum_required(VERSION 3.5)
project(virtual_data_publisher)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加源代码目录
add_executable(virtual_data_publisher src/virtual_data_publisher.cpp)

# 链接依赖库
ament_target_dependencies(virtual_data_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  virtual_data_publisher
  DESTINATION lib/${PROJECT_NAME}
)


# 在build目录中运行测试
ament_package()
