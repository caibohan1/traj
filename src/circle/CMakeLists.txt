cmake_minimum_required(VERSION 3.5)
project(circle)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# 包含 Eigen 头文件
include_directories(${EIGEN3_INCLUDE_DIRS})

# 1. 编译 Trajectory Planner 节点
add_executable(trajectory_planner src/circlecontrol.cpp)
ament_target_dependencies(trajectory_planner rclcpp px4_msgs nav_msgs geometry_msgs std_msgs)

# 2. 编译 Geometric Controller 节点
add_executable(geometric_controller src/GeometricController.cpp)
ament_target_dependencies(geometric_controller rclcpp px4_msgs Eigen3 std_msgs)

# 3. 编译新增的 8字形轨迹 节点 (你的新文件)
add_executable(eight_planner src/eightcontrol.cpp)
ament_target_dependencies(eight_planner rclcpp px4_msgs nav_msgs geometry_msgs std_msgs)

# 1. 添加控制器节点
add_executable(controller_node src/Controller.cpp)
ament_target_dependencies(controller_node rclcpp px4_msgs Eigen3 std_msgs)

# mav轨迹规划器节点
add_executable(mavcircle_planner src/Mav_circlecontrol.cpp)
ament_target_dependencies(mavcircle_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

# mav轨迹规划器节点
add_executable(maveight_planner src/Mav_eightcontrol.cpp)
ament_target_dependencies(maveight_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

# 几何控制器节点
add_executable(mavController src/Mav_Controller.cpp)
ament_target_dependencies(mavController
  rclcpp
  std_msgs
  nav_msgs
  mavros_msgs
)
target_link_libraries(geometric_controller Eigen3::Eigen) # 链接 Eigen3
# 安装可执行文件
install(TARGETS
  trajectory_planner
  geometric_controller
  eight_planner
  controller_node
  mavcircle_planner
  maveight_planner
  mavController
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
