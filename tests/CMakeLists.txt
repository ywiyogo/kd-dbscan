include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/bf66935e07825318ae519675d73d0f3e313b3ec6.zip                                  
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(UNITTEST_SOURCES
    kdtree_test.cpp
    dbscan_test.cpp)

add_executable(kdtree_test kdtree_test.cpp)
add_executable(dbscan_test dbscan_test.cpp)
include_directories(dbscan_test ${CMAKE_SOURCE_DIR}/include)
target_include_directories(kdtree_test INTERFACE ${CMAKE_SOURCE_DIR}/include/)

target_link_libraries(kdtree_test gtest_main)
target_link_libraries(dbscan_test gtest_main)


add_test(NAME kdtree_test COMMAND kdtree_test)
add_test(NAME dbscan_test COMMAND dbscan_test)
