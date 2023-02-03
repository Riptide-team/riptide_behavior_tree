cmake_minimum_required(VERSION 3.8)
project(riptide_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(riptide_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  std_msgs
  std_srvs
  nav2_util
  riptide_msgs
  controller_manager_msgs
)

# Behavior tree engine
add_executable(${library_name} src/behavior_tree_engine.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# Control
add_library(recovery_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)

add_library(say_something SHARED plugins/control/say_something.cpp)
list(APPEND plugin_libs say_something)

# Condition
add_library(is_battery_lower_than SHARED plugins/condition/is_battery_lower_than.cpp)
list(APPEND plugin_libs is_battery_lower_than)

add_library(is_pressure_greater_than SHARED plugins/condition/is_pressure_greater_than.cpp)
list(APPEND plugin_libs is_pressure_greater_than)

# Decorator
add_library(inverter_node SHARED plugins/decorator/inverter_node.cpp)
list(APPEND plugin_libs inverter_node)

# Service
add_library(load_controller SHARED plugins/service/load_controller.cpp)
list(APPEND plugin_libs load_controller)

# Adding dependencies
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install (TARGETS ${library_name} ${plugin_libs}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt
  DESTINATION share/${PROJECT_NAME}
)

install(FILES riptide_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
