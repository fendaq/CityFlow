cmake_minimum_required(VERSION 3.5)

project(Simulator)

#set(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(CMAKE_CXX_STANDARD 14)
if (APPLE)
	message(STATUS "macOS detected")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall")
elseif (UNIX AND NOT APPLE)
	message(STATUS "Linux detected")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall")
endif ()

include_directories(
		include
)

# dependecncies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

set(Boost_USE_STATIC_LIBS FALSE)
find_package(Boost REQUIRED
		COMPONENTS thread log unit_test_framework)
include_directories(${Boost_INCLUDE_DIR})
add_definitions(
#		-D BOOST_THREAD_VERSION=4
)

add_subdirectory(src)
add_subdirectory(test)

file(GLOB_RECURSE LIB_SOURCE *.cpp *.cc *.h)

message(STATUS ${LIB_SOURCE})

