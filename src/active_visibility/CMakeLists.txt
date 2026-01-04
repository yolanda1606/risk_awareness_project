cmake_minimum_required(VERSION 3.8)
project(active_visibility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 1. CRITICAL FIX: Find the Google Logging Library
find_package(glog REQUIRED) 

add_executable(inspector_node src/inspector_node.cpp)

ament_target_dependencies(inspector_node 
  rclcpp 
  voxblox 
  voxblox_ros 
  voxblox_msgs
  visualization_msgs
  std_msgs
)

# 2. CRITICAL FIX: Link glog explicitly to resolve 'undefined reference' errors
target_link_libraries(inspector_node glog::glog)

install(TARGETS inspector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()