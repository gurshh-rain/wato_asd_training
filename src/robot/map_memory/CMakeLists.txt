cmake_minimum_required(VERSION 3.10)
project(map_memory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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

add_library(map_memory_lib
  src/map_memory_core.cpp)
target_include_directories(map_memory_lib
  PUBLIC include)
ament_target_dependencies(map_memory_lib
  rclcpp nav_msgs geometry_msgs)

add_executable(map_memory_node src/map_memory_node.cpp)
target_link_libraries(map_memory_node map_memory_lib)
ament_target_dependencies(map_memory_node
  rclcpp nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  map_memory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_memory test/test_map_memory.cpp)
  target_link_libraries(test_map_memory map_memory_lib)
  ament_target_dependencies(test_map_memory rclcpp nav_msgs geometry_msgs)
endif()

ament_package()
