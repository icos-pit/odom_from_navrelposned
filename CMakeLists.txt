cmake_minimum_required(VERSION 3.8)
project(odom_from_navrelposned)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(gps_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
  include
)

add_executable(odom_from_navrelposned src/main.cpp src/dgps.cpp)

target_include_directories(odom_from_navrelposned PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(odom_from_navrelposned PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(odom_from_navrelposned)

ament_target_dependencies(
  odom_from_navrelposned
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "rclpy"
  "sensor_msgs"
  "std_msgs"
  "visualization_msgs"
  "ublox_msgs"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "Eigen3"
)

install(TARGETS odom_from_navrelposned
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
