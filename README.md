# Dev setup

## Prerequisites
Supported OS: Windows 10/11 | Ubuntu 24.04 LTS
- CMake (Latest)
- qt5 (`vcpkg install qt5` | `sudo apt install qtbase5-dev`)
- spdlog (`vcpkg install spdlog` | `sudo apt install libspdlog-dev`)
- CGAL ([windows](https://doc.cgal.org/5.6.1/Manual/windows.html) | `sudo apt-get install libcgal-dev`)
- cxx build tools ([VS2019 community](https://www.techspot.com/downloads/7241-visual-studio-2019.html) | `sudo apt-get install build-essential gdb`)

## Clone
```
git clone https://github.com/guotata1996/RoadRunner.git
git submodule init && git submodule update
```

## Build
### Windows
- Follow [CGAL Configuring example](https://doc.cgal.org/5.6.1/Manual/windows.html) to configure CMake
- Open the sln with Visual Studio
- Right click RoadRunner -> build

### Ubuntu
```
cd RoadRunner
mkdir build
cmake ..
make
```
