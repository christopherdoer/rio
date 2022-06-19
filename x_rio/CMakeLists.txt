cmake_minimum_required(VERSION 2.8.3)
project(x_rio)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_compile_options(-std=c++14)

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=native")

cs_add_library(${PROJECT_NAME}
   src/data_types.cpp
   src/x_rio_ros.cpp
   src/x_rio_filter.cpp
   src/yaw_aiding_manhattan_world.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})


cs_add_executable(ros_node src/nodes/ros_node.cpp )
target_link_libraries(ros_node ${PROJECT_NAME})

cs_add_executable(rosbag_node src/nodes/rosbag_node.cpp )
target_link_libraries(rosbag_node ${PROJECT_NAME})


cs_install()

cs_export(INCLUDE_DIRS include)


## qtcreator project tree
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

file(GLOB_RECURSE CFG "cfg/*")
add_custom_target(CFG SOURCES ${CFG})

file(GLOB_RECURSE MSG "msg/*")
add_custom_target(MSG SOURCES ${MSG})

file(GLOB_RECURSE SRV "srv/*")
add_custom_target(SRV SOURCES ${SRV})

file(GLOB_RECURSE LAUNCH "launch/*")
add_custom_target(LAUNCH SOURCES ${LAUNCH})

file(GLOB_RECURSE PYTHON "python/*")
add_custom_target(PYTHON SOURCES ${PYTHON})

file(GLOB_RECURSE CONFIG "config/*")
add_custom_target(CONFIG SOURCES ${CONFIG})
