project(libPF)
cmake_minimum_required(VERSION 2.8)

include_directories(include)

add_library(PF SHARED
src/CRandomNumberGenerator.cpp)

set_property(TARGET PF PROPERTY POSITION_INDEPENDENT_CODE ON)
