cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(cmakeSamples)

include_directories(${PROJECT_SOURCE_DIR}
  /usr/local/include/urg_cpp)

link_directories(${PROJECT_SOURCE_DIR}
  /usr/local/lib)

add_executable (main main.cpp Connection_information.cpp)
target_link_libraries(main ncursesw urg_cpp)
