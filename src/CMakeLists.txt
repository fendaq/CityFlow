include_directories(
		${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB_RECURSE LIB_SOURCE *.cpp *.cc *.h)

add_subdirectory(../pybind11 pybind11.out)
pybind11_add_module(cityflow SHARED EXCLUDE_FROM_ALL ${LIB_SOURCE} ${INCLUDE_DIRECTORIES})
target_link_libraries(cityflow PRIVATE ${Boost_LIBRARIES})

list(REMOVE_ITEM LIB_SOURCE ${CMAKE_CURRENT_LIST_DIR}/simulator.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_SOURCE} ${INCLUDE_DIRECTORIES})
target_link_libraries(
		${PROJECT_NAME}
		${Boost_LIBRARIES}
)
