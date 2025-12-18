
cmake_minimum_required(VERSION 3.8)
project(robosense_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# 集成 rs_driver：添加子目录并查找包
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)

# 查找 rs_driver 包（它会设置 rs_driver_INCLUDE_DIRS）
find_package(rs_driver REQUIRED)

# 添加可执行文件
add_executable(rsview src/rsview.cpp)

# 设置节点依赖
ament_target_dependencies(rsview
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

# 包含 rs_driver 的头文件目录
include_directories(
    ${rs_driver_INCLUDE_DIRS}      # rs_driver 的头文件路径
)

# 如果 rs_driver 提供了库（如 librs_driver.so），链接它
# 注意：add_subdirectory 已经构建了库，这里只需链接
target_link_libraries(rsview
    ${rs_driver_LIBRARIES}
)

# 安装目标
install(TARGETS rsview
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件（可选）
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
