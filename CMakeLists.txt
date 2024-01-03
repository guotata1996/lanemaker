cmake_minimum_required(VERSION 3.1...3.23)
project(RoadRunner)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

add_subdirectory(libOpenDRIVE-master)
add_executable(${CMAKE_PROJECT_NAME} main.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/include
    ${CMAKE_SOURCE_DIR}/libOpenDRIVE-master/thirdparty)

target_link_libraries(${CMAKE_PROJECT_NAME} 
    Qt5::Core Qt5::Gui Qt5::Widgets 
    CGAL::CGAL 
    OpenDrive
)

