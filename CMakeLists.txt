cmake_minimum_required( VERSION 3.18 )
project( kd-dbscan VERSION 1.0 DESCRIPTION "DBSCAN using KD-Tree" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

add_library(kd-dbscan INTERFACE)
target_include_directories(kd-dbscan INTERFACE include/)

add_executable(run_app main.cpp)
set_target_properties(run_app PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(run_app PRIVATE kd-dbscan)

enable_testing()    # it is enough to write this function once in this root file to include test cases in CTest
add_subdirectory(tests)
