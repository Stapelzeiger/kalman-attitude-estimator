cmake_minimum_required(VERSION 2.6)

project(flight-stack)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories(src)
include_directories(lib)
include_directories(lib/eigen)
include_directories(lib/cppzmq)

file(GLOB_RECURSE src src/*.cpp tmp/*.cpp)

add_executable(
    tests
    ${src}
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )


add_executable(
    example
    test/test.cpp
    test/state_estimator.cpp
    lib/cmp/cmp.c
    lib/cmp_mem_access/cmp_mem_access.c
    )

target_link_libraries(
    example
    zmq
    )
