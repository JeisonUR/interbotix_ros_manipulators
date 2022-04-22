cmake_minimum_required(VERSION 3.5)
project(raya_cv_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detections2D.msg"
  "msg/DetectionAT.msg"
  "msg/DetectionsAT.msg"
  "msg/Recognition2D.msg"
  "msg/Recognitions2D.msg"
  "srv/GetAvailableModels.srv"
  "srv/EnableModel.srv"
  "srv/DisableModel.srv"
  "srv/DisableAllModels.srv"
  "srv/SetPclId.srv"
  "srv/SetPclName.srv"
  "srv/GetGripperPoses.srv"
  "srv/EnablePointcloud.srv"
  "srv/TrainModel.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
