cmake_minimum_required(VERSION 3.8)
project(gen3_lite_examples_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dynamic_model
  src/dynamic_model.cpp
)

target_include_directories(dynamic_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_model)

add_executable(gravity_cancellation_node src/gravity_cancellation_node.cpp)

target_link_libraries(gravity_cancellation_node
  dynamic_model
)

ament_target_dependencies(gravity_cancellation_node 
                          rclcpp 
                          sensor_msgs 
                          std_msgs)

install(TARGETS
  dynamic_model
  gravity_cancellation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
