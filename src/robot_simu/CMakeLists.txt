cmake_minimum_required(VERSION 3.8)
project(robot_simu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(gazebo REQUIRED) # For simulation
#find_package(gazebo_ros_pkgs REQUIRED) # For simulation
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
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

install(
    DIRECTORY config launch worlds rviz
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
    launch/launch_sim.launch.py
    launch/rsp.launch.py
    launch/rviz.launch.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
#ament_export_dependencies(gazebo_ros_pkgs) # For simulation
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
