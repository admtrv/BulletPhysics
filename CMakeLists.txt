# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(BulletPhysics)

set(CMAKE_CXX_STANDARD 20)

# library sources
set(LIB_NAME ${PROJECT_NAME})

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX ".*/.*build.*/.*")
list(FILTER SOURCES EXCLUDE REGEX ".*/tests/.*")

add_library(${LIB_NAME} STATIC ${SOURCES})

# public includes
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# tests
add_subdirectory(tests)
