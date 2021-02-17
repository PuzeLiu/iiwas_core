cmake_minimum_required(VERSION 3.4...3.18)
project(iiwas_kinematics)

find_package(Eigen3 REQUIRED)

add_subdirectory(pybind11)
pybind11_add_module(iiwas_kinematics src/python_bindings.cpp src/iiwas_kinematics.cpp)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_include_directories(iiwas_kinematics PRIVATE include)
target_compile_definitions(iiwas_kinematics PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
