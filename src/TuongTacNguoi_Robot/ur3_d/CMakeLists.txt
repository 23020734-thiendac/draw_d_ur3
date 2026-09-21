cmake_minimum_required(VERSION 3.8)
project(ur3_d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(ur3_d_core
  src/end_effector_tracer.cpp
  src/letter_d_path.cpp
  src/moveit_executor.cpp
)
target_include_directories(ur3_d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ur3_d_core
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_executable(write_letter_d_node src/write_letter_d_node.cpp)
target_link_libraries(write_letter_d_node ur3_d_core)
ament_target_dependencies(write_letter_d_node
  ament_index_cpp
  rclcpp
  moveit_ros_planning_interface
  visualization_msgs
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(PROGRAMS scripts/simulation_ready.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ur3_d_core
  write_letter_d_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
