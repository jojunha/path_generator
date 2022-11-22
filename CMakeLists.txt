cmake_minimum_required(VERSION 3.0.2)
project(path_generator)
add_compile_options(-std=c++11 -O)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf
    visualization_msgs 
)

find_package(Eigen3 3.3 REQUIRED)
find_package(ompl REQUIRED)

set(SRC_FILES
    src/motionplan.cpp
    src/visualization.cpp
)
    
message("------------------------------------")
message("OMPL Path : " ${OMPL_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES path_generator
  CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

add_library(path_lib
    ${SRC_FILES}
)

add_executable(path_generator
   src/main.cpp
   src/motionplan.cpp
   src/visualization.cpp
)

target_link_libraries(path_generator
PUBLIC
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   Eigen3::Eigen
   path_lib
)
