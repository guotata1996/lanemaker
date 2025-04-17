cmake_minimum_required(VERSION 3.1...3.23)
project(LaneMaker VERSION 0.9.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(spdlog REQUIRED)

add_subdirectory(libOpenDRIVE-master)

qt5_add_resources(srcs_for_exe ui/images.qrc)
qt5_add_resources(srcs_for_exe engine/shaders.qrc)
set(APP_ICON_RESOURCE_WINDOWS "${CMAKE_CURRENT_SOURCE_DIR}/lanemaker.rc")

add_executable(${CMAKE_PROJECT_NAME} main.cpp ${APP_ICON_RESOURCE_WINDOWS}
    xodr/road.cpp xodr/road_operation.cpp xodr/curve_fitting.cpp xodr/polyline.cpp
    xodr/junction.cpp xodr/junction_generation.cpp xodr/junction_boundary.cpp
    xodr/id_generator.cpp xodr/change_tracker.cpp xodr/world.cpp
    ui/main_window.cpp ui/main_widget.cpp ${srcs_for_exe}
    ui/road_graphics.cpp ui/road_drawing.cpp ui/road_creation.cpp ui/lane_creation.cpp
    ui/road_modification.cpp ui/road_destruction.cpp ui/road_overlaps.cpp
    ui/CreateRoadOptionWidget.cpp ui/action_manager.cpp ui/util.cpp ui/replay_window.cpp
    ui/touch_controller.cpp
    engine/map_view_gl.cpp engine/ShaderProgram.cpp 
    engine/Transform3D.cpp engine/gl_buffer_manage.cpp engine/gl_buffer_manage_instanced.cpp
    engine/spatial_indexer.cpp engine/spatial_indexer_dynamic.cpp
    traffic/vehicle.cpp traffic/vehicle_manager.cpp traffic/signal.cpp
    util/stats.cpp util/multi_segment.cpp util/label_with_link.cpp util/preference.cpp
    util/triangulation.cpp
    test/validation.cpp test/junction_validation.cpp test/road_validation.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    xodr ui engine util traffic
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

# prevent CGAL warnings from crashing release builds
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CGAL_DEBUG)


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
  LaneMakerTest
  test/test.cc test/randomization_utils.cpp 
  test/validation.cpp test/junction_validation.cpp test/road_validation.cpp
  xodr/road.cpp xodr/road_operation.cpp xodr/curve_fitting.cpp xodr/polyline.cpp
  xodr/junction.cpp xodr/junction_generation.cpp
  xodr/id_generator.cpp xodr/world.cpp
)

target_include_directories(LaneMakerTest PRIVATE
    xodr
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/include
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/thirdparty
)

target_link_libraries(
  LaneMakerTest
  GTest::gtest_main
  CGAL::CGAL
  OpenDrive
  spdlog::spdlog
)

target_compile_features(LaneMakerTest PRIVATE cxx_std_17)

include(GoogleTest)
gtest_discover_tests(LaneMakerTest)

target_compile_definitions(LaneMakerTest PRIVATE G_TEST)

# ====================================
# Pack DEBIAN
# ====================================

set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME}
    CACHE STRING "The resulting package name"
)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LaneMaker Release"
    CACHE STRING "Package description for the package metadata"
)
set(CPACK_PACKAGE_VENDOR "guotata1996")

set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}/build/packages")

# set(CPACK_PACKAGING_INSTALL_PREFIX "/opt")

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

set(CPACK_PACKAGE_CONTACT "guotata@gmail.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "guotata")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

# package name for deb. If set, then instead of some-application-0.9.2-Linux.deb
# you'll get some-application_0.9.2_amd64.deb (note the underscores too)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
# that is if you want every group to have its own package,
# although the same will happen if this is not set (so it defaults to ONE_PER_GROUP)
# and CPACK_DEB_COMPONENT_INSTALL is set to YES
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)#ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL YES)

set(CPACK_DEBIAN_PACKAGE_DEPENDS "qtbase5-dev, libspdlog-dev, libcgal-dev(<=5.6.2)")

include(CPack)
install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION bin RENAME lanemaker)