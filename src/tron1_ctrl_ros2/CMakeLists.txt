cmake_minimum_required(VERSION 3.8)
project(tron1_ctrl_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(limxsdk_lowlevel REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json 3 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/SetBaseHeight.srv"
  "srv/SetLightEffect.srv"
  DEPENDENCIES std_msgs
)

add_executable(tron1_node src/tron1_node.cpp)
ament_target_dependencies(tron1_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  nlohmann_json
)

rosidl_target_interfaces(tron1_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# websocketpp is header only, just link boost system
target_link_libraries(tron1_node
  ${Boost_LIBRARIES}
  pthread
  ssl
  crypto
)

add_executable(limxsdk_teleop src/limxsdk_teleop_node.cpp)
ament_target_dependencies(limxsdk_teleop
  rclcpp
  geometry_msgs
  limxsdk_lowlevel
)

add_executable(websocket_cmd_vel_bridge src/websocket_cmd_vel_bridge.cpp)
ament_target_dependencies(websocket_cmd_vel_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  nlohmann_json
)
target_link_libraries(websocket_cmd_vel_bridge
  ${Boost_LIBRARIES}
  pthread
  ssl
  crypto
)

install(TARGETS
  tron1_node
  limxsdk_teleop
  websocket_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
