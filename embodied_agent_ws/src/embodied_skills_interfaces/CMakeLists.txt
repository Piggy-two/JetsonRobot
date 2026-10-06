cmake_minimum_required(VERSION 3.8)
project(embodied_skills_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRelative.srv"
  "srv/Rotate.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
