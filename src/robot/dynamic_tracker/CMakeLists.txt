cmake_minimum_required(VERSION 3.10)
project(dynamic_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  foreach(_target rclcpp::rclcpp rcutils::rcutils)
    if(TARGET ${_target})
      get_target_property(_interface_libs ${_target} INTERFACE_LINK_LIBRARIES)
      if(_interface_libs)
        list(REMOVE_ITEM _interface_libs "atomic" "-latomic")
        set_target_properties(${_target} PROPERTIES INTERFACE_LINK_LIBRARIES "${_interface_libs}")
      endif()
    endif()
  endforeach()
endif()

add_library(dynamic_tracker_lib src/dynamic_tracker_core.cpp)
target_include_directories(dynamic_tracker_lib PUBLIC include)
ament_target_dependencies(dynamic_tracker_lib
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs)

add_executable(dynamic_tracker_node src/dynamic_tracker_node.cpp)
target_link_libraries(dynamic_tracker_node dynamic_tracker_lib)
ament_target_dependencies(dynamic_tracker_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros)

install(TARGETS dynamic_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_tracker test/test_dynamic_tracker.cpp)
  target_link_libraries(test_dynamic_tracker dynamic_tracker_lib)
  ament_target_dependencies(test_dynamic_tracker
    rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs)
endif()

ament_package()
