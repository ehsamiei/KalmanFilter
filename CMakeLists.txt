cmake_minimum_required(VERSION 3.1.0)
project(myproject)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_executable(myproject kalman.cpp main.cpp)


find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(myproject PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(myproject Python3::Python Python3::NumPy)


