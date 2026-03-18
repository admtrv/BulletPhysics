# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)

find_package(GTest REQUIRED)

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(BulletPhysicsTests ${TEST_SOURCES})
target_link_libraries(BulletPhysicsTests PRIVATE BulletPhysics GTest::GTest GTest::Main)

include(GoogleTest)
gtest_discover_tests(BulletPhysicsTests)
