cmake_minimum_required(VERSION 3.1...3.23)
project(RoadRunner)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(spdlog REQUIRED)

add_subdirectory(libOpenDRIVE-master)
# TODO: make RoadRunner a library

qt5_add_resources(srcs_for_exe ui/images.qrc) 
set(APP_ICON_RESOURCE_WINDOWS "${CMAKE_CURRENT_SOURCE_DIR}/roadrunner.rc")

add_executable(${CMAKE_PROJECT_NAME} main.cpp ${APP_ICON_RESOURCE_WINDOWS}
    xodr/road.cpp xodr/road_operation.cpp xodr/curve_fitting.cpp xodr/polyline.cpp
    xodr/junction.cpp xodr/junction_generation.cpp xodr/junction_boundary.cpp
    xodr/id_generator.cpp xodr/change_tracker.cpp xodr/world.cpp
    xodr/spatial_indexer.cpp
    ui/mainwindow.cpp ui/main_widget.cpp ui/map_view.cpp ${srcs_for_exe}
    ui/road_graphics.cpp ui/road_drawing.cpp ui/road_creation.cpp ui/lane_creation.cpp
    ui/road_modification.cpp ui/road_destruction.cpp ui/road_overlaps.cpp
    ui/CreateRoadOptionWidget.cpp ui/action_manager.cpp ui/util.cpp ui/replay_window.cpp
    ui/OpenGLWindow.cpp ui/map_view_gl.cpp ui/ShaderProgram.cpp ui/Transform3D.cpp
    traffic/vehicle.cpp traffic/vehicle_manager.cpp traffic/signal.cpp
    util/stats.cpp util/multi_segment.cpp util/label_with_link.cpp util/preference.cpp
    util/triangulation.cpp
    test/validation.cpp test/junction_validation.cpp test/road_validation.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    xodr ui util traffic
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/include
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/thirdparty
    cereal/include)

target_link_libraries(${CMAKE_PROJECT_NAME} 
    Qt5::Core Qt5::Gui Qt5::Widgets 
    CGAL::CGAL 
    OpenDrive
    spdlog::spdlog
)

target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_17)


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
  test/validation.cpp test/junction_validation.cpp test/road_validation.cpp
  xodr/road.cpp xodr/road_operation.cpp xodr/curve_fitting.cpp xodr/polyline.cpp
  xodr/junction.cpp xodr/junction_generation.cpp
  xodr/id_generator.cpp xodr/world.cpp
)

target_include_directories(RoadRunnerTest PRIVATE
    xodr
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

target_compile_features(RoadRunnerTest PRIVATE cxx_std_17)

include(GoogleTest)
gtest_discover_tests(RoadRunnerTest)

target_compile_definitions(RoadRunnerTest PRIVATE G_TEST)
