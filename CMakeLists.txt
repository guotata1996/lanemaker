cmake_minimum_required(VERSION 3.1...3.23)
project(RoadRunner)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(spdlog REQUIRED)

add_subdirectory(libOpenDRIVE-master)
# TODO: make RoadRunner a library

qt5_add_resources(srcs_for_exe images.qrc) 

add_executable(${CMAKE_PROJECT_NAME} main.cpp 
    roadProfile.cpp road.cpp junction.cpp road_operation.cpp curve_fitting.cpp
    IDGenerator.cpp MapExporter.cpp world.cpp
    mainwindow.cpp view.cpp ${srcs_for_exe} RoadGraphics.cpp
    road_drawing.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/include
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/thirdparty)

target_link_libraries(${CMAKE_PROJECT_NAME} 
    Qt5::Core Qt5::Gui Qt5::Widgets 
    CGAL::CGAL 
    OpenDrive
    spdlog::spdlog
)

# ====================================
# Google Test
# ====================================

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  RoadRunnerTest
  test/test.cc test/randomization_utils.cpp
  roadProfile.cpp road.cpp junction.cpp road_operation.cpp curve_fitting.cpp
  IDGenerator.cpp MapExporter.cpp
)

target_include_directories(RoadRunnerTest PRIVATE
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/include
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/thirdparty
)

target_link_libraries(
  RoadRunnerTest
  GTest::gtest_main
  CGAL::CGAL
  OpenDrive
  spdlog::spdlog
)

include(GoogleTest)
gtest_discover_tests(RoadRunnerTest)

target_compile_definitions(RoadRunnerTest PRIVATE G_TEST)
