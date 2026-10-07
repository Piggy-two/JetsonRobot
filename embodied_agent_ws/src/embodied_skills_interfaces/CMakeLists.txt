cmake_minimum_required(VERSION 3.8)
project(embodied_skills_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# SkillEvent.msg 带时间戳字段，需要 builtin_interfaces/Time
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRelative.srv"
  "srv/Rotate.srv"
  "srv/SectorMinRange.srv"
  "srv/PathClear.srv"
  "srv/SkillInvoke.srv"
  "srv/SkillResult.srv"
  "srv/SkillCancel.srv"
  "srv/SkillList.srv"
  "msg/SkillEvent.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
