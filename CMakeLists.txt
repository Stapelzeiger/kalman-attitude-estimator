cmake_minimum_required(VERSION 2.6)

project(flight-stack)
set(CMAKE_BUILD_TYPE Debug)

include_directories(src)
include_directories(lib/eigen)

file(GLOB_RECURSE src src/*.cpp)

add_executable(
    tests
    ${src}
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )
