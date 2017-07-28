project(FilteringExercise)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(Eigen3 REQUIRED)

file(GLOB INCLUDE
    include/*.h
)

file(GLOB SRC
    src/*.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} ${SRC} ${INCLUDE})
