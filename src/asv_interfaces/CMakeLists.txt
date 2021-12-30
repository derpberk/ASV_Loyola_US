cmake_minimum_required(VERSION 3.5)
project(asv_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Status.msg"
  #"msg/Message2.msg"
  # etc
  )


set(srv_files
  "srv/Newwaypoint.srv"
  #"srv/Service2.srv"
   # etc
  )

set(action_files
  "action/Example.action"
  #"action/Action2.action"
   # etc
  )


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
 )




ament_package()
