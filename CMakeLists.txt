cmake_minimum_required(VERSION 3.14)
project(ros2_lucid)
include(FetchContent)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wno-unused-parameter")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
  
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(fmt REQUIRED)

set(ros_dependencies
  rclcpp
  std_msgs
  std_srvs
  lifecycle_msgs
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  arena_sdk
)
  
  set(other_dependencies
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  nlohmann_json::nlohmann_json
  rerun_sdk
  fmt::fmt
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraDevice.msg"
  "msg/CameraDeviceArray.msg"
  "srv/AskCamera.srv"
  "srv/TriggerCamera.srv"
  "srv/TriggerCapture.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(camera_discover src/CameraDiscover.cpp)
ament_target_dependencies(camera_discover ${ros_dependencies})
target_link_libraries(camera_discover "${cpp_typesupport_target}")
target_link_libraries(camera_discover ${other_dependencies})

add_executable(camera_node src/CameraNode.cpp)
ament_target_dependencies(camera_node ${ros_dependencies})
target_link_libraries(camera_node "${cpp_typesupport_target}")
target_link_libraries(camera_node ${other_dependencies})

add_executable(camera_manager src/CameraManager.cpp)
ament_target_dependencies(camera_manager ${ros_dependencies})
target_link_libraries(camera_manager "${cpp_typesupport_target}")
target_link_libraries(camera_manager ${other_dependencies})


install(TARGETS
  camera_discover
  camera_node
  camera_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
