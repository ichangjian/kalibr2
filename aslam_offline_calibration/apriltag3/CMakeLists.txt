cmake_minimum_required(VERSION 2.8)

project(apriltag3)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
catkin_package(
    DEPENDS OpenCV
    INCLUDE_DIRS include .
    LIBRARIES ${PROJECT_NAME}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

aux_source_directory(common COMMON_SRC)
add_definitions(-fPIC -O3)
include_directories(include  . ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.c")
add_library(${PROJECT_NAME}  ${SOURCE_FILES} ${COMMON_SRC} )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#demo
if(NOT APPLE)
    add_executable(aprilgrid_demo example/aprilgrid_demo.cc)
    target_link_libraries(aprilgrid_demo ${PROJECT_NAME} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})
endif()

