cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_fusion
  CATKIN_DEPENDS robot_localization roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ 노드 추가 (옵션)
# 만약 C++로 센서 융합 노드를 작성할 경우, 다음을 추가합니다.
# add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
# add_dependencies(sensor_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sensor_fusion_node
#   ${catkin_LIBRARIES}
# )

## Python 스크립트 설치
 catkin_install_python(PROGRAMS
  scripts/sensor_fusion_node.py
  scripts/path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

