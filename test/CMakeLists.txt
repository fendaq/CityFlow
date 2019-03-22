include_directories(../src)

file(GLOB_RECURSE TEST_SOURCE ${CMAKE_CURRENT_LIST_DIR}/test.cpp)


#file(GLOB_RECURSE TMP ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
#set(TEST_SOURCE ${TEST_SOURCE} ${TMP})

add_executable(${PROJECT_NAME}_test ${TEST_SOURCE})

# boost
find_package(Boost REQUIRED
		COMPONENTS unit_test_framework thread serialization log program_options
		)
include_directories(${Boost_INCLUDE_DIR})
add_definitions(
#		-D BOOST_THREAD_VERSION=4
		-D BOOST_LOG_DYN_LINK
)

# link
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${Boost_LIBRARIES})

# config
add_definitions(-DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")