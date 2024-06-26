cmake_minimum_required(VERSION 3.5)
project(moveit2_scripts)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pick_place src/pick_and_place.cpp)
add_executable(pick_place_v2 src/pick_and_place_v2.cpp)

target_include_directories(pick_place
    PUBLIC include)
ament_target_dependencies(pick_place
	ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	control_msgs
	moveit_msgs
	geometry_msgs

)
ament_target_dependencies(pick_place_v2
	ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	control_msgs
	moveit_msgs
	geometry_msgs
	grasping_msgs

)
install(TARGETS

	pick_place
	pick_place_v2
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)
ament_package()
