cmake_minimum_required(VERSION 2.6)

project(flight-stack)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories(src)
include_directories(lib)
include_directories(lib/eigen)
include_directories(lib/cppzmq)

file(GLOB_RECURSE src src/*.cpp)

add_executable(
    tests
    ${src}
    attitude_estimation/ekf_gyro_acc.cpp
    attitude_estimation/ekf_gyro_acc_test.cpp
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )


add_executable(
    run_ekf_zmq
    attitude_estimation/run_estimator_on_zmq.cpp
    attitude_estimation/ekf_gyro_acc.cpp
    lib/cmp/cmp.c
    lib/cmp_mem_access/cmp_mem_access.c
    )

target_link_libraries(
    run_ekf_zmq
    zmq
    )
