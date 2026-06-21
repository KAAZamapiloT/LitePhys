cmake_minimum_required(VERSION 3.10)
project(LitePhys)

set(CMAKE_CXX_STANDARD 11)

# Add include directory for the library
include_directories(include)

# Executables for examples
add_executable(example_gravity examples/example_gravity.cpp)
add_executable(example_gravity_4d examples/example_gravity_4d.cpp)
add_executable(example_optics_3d examples/example_optics_3d.cpp)
add_executable(example_optics_4d examples/example_optics_4d.cpp)
