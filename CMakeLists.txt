cmake_minimum_required( VERSION 3.7 )
project(entity_component_event)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")
endif(MSVC)

find_package(PCL REQUIRED)

find_package(GTest REQUIRED)
message(=============================================================)
message("-- option COMPILE_TEST switch on")
message("-- gtest header: ${GTEST_INCLUDE_DIR}")
message("-- gtest libraries: ${GTEST_BOTH_LIBRARIES}")
message(=============================================================)


include_directories(${GTEST_INCLUDE_DIR})


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})


FILE(GLOB_RECURSE SRC_LISTS ${CMAKE_SOURCE_DIR}/src/*.cpp)



add_executable(component_test ${CMAKE_SOURCE_DIR}/test/component_test.cpp)
add_executable(factory_test ${CMAKE_SOURCE_DIR}/test/factory_test.cpp)
add_executable(test test/test.cpp)
target_link_libraries(test ${GTEST_BOTH_LIBRARIES}  pthread )

