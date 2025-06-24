cmake_minimum_required(VERSION 3.0.2)
project(oak_ffc_4p_ros)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wno-write-strings -Wno-format-overflow ")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(depthai_ros_msgs_DIR /home/pml-omni/dai_ws/devel/share/depthai_ros_msgs/cmake/)
set(depthai_bridge_DIR /home/pml-omni/dai_ws/devel/share/depthai_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  depthai_ros_msgs
  depthai_bridge
  dynamic_reconfigure
  # depthai
)

generate_dynamic_reconfigure_options(
  cfg/dynConfig.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myu_pkg
 CATKIN_DEPENDS roscpp dynamic_reconfigure
#  DEPENDS system_lib
)

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

#Find opencv
set(OpenCV_DIR  /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4 REQUIRED )
# find_package(Sanitizers)

#Find depthai SDK
find_package(depthai CONFIG REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# 更改为对应的路径
link_directories(
  # /home/pml-omni/3rdParty/depthai-core/build/
  /usr/local/lib
  /home/pml-omni/3rdParty/XLink/build
  /home/pml-omni/3rdParty/spdlog/build
  /home/pml-omni/3rdParty/fmt/build
)

###################
add_executable(${PROJECT_NAME}_test test.cpp)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::opencv
  depthai-core
  XLink
  spdlog
  fmt::fmt
  pthread
)

target_include_directories(${PROJECT_NAME}_test PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # /home/pml-omni/3rdParty/depthai-core/include
  /usr/local/include
  /home/pml-omni/3rdParty/XLink/build/install/include
)

#####################3
add_library(${PROJECT_NAME}_lib STATIC
  oak_ffc_4p.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::opencv
  depthai-core
  XLink
  spdlog
  fmt::fmt
  pthread
)

add_executable(${PROJECT_NAME}_node oak_ffc_4p_node.cpp)
# add_dependencies(${PROJECT_NAME}_node   ${PROJECT_NAME}_lib  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::opencv
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

