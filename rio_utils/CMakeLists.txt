cmake_minimum_required(VERSION 2.8.3)
project(rio_utils)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_compile_options(-std=c++14)

cs_add_library(${PROJECT_NAME}
  src/strapdown.cpp
  src/simple_profiler.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread)

cs_install()

cs_export(INCLUDE_DIRS include)

#############
# QTCREATOR #
#############
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
