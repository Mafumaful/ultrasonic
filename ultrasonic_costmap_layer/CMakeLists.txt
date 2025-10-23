cmake_minimum_required(VERSION 3.5)
project(ultrasonic_costmap_layer)

set(lib_name ${PROJECT_NAME}_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(car_chassis REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  nav2_costmap_2d
  pluginlib
  car_chassis
  tf2
  tf2_ros
)

# Build library
add_library(${lib_name} SHARED
  src/ultrasonic_layer.cpp
)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${lib_name} ${dependencies})

# Install library
install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install config and test files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test/
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

# Export plugin description file
pluginlib_export_plugin_description_file(nav2_costmap_2d ultrasonic_layer.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dependencies})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
