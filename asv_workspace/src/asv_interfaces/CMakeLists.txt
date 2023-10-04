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
  "msg/Nodeupdate.msg"
  "msg/String.msg"
  "msg/Samplevalue.msg"
  "msg/Location.msg"
  "msg/Sensor.msg"
  "msg/Threeaxys.msg"
  "msg/Camera.msg"
  "msg/Obstacles.msg"
  "msg/Sonar.msg"

  #"msg/Message2.msg"
  # etc
  )


set(srv_files
  "srv/Newpoint.srv"
  "srv/ASVmode.srv"
  "srv/CommandBool.srv"
  "srv/Newrelativepoint.srv"
  "srv/Takesample.srv"
  "srv/LoadMission.srv"
  "srv/CommandStr.srv"
  "srv/PlannerParams.srv"
  "srv/SensorParams.srv"
  "srv/SonarService.srv"
  "srv/SensorService.srv"
  #"srv/Service2.srv"
   # etc
  )

set(action_files
  "action/Newmission.action"
  "action/Samplepoint.action"
  "action/Goto.action"
  "action/SensorSample.action"
  #"action/Action2.action"
   # etc
  )


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
 )




ament_package()
