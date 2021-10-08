project(LASFILTER CXX)

file(GLOB HEADERS "${PROJECT_SOURCE_DIR}/include/*.h")
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

set(SRCS_FILES ${HEADERS} ${SOURCES})

add_executable(${PROJECT_NAME} ${SRCS_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/LASlib/include ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC LASlib)


