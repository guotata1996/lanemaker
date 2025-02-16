![CMake Build](https://github.com/guotata1996/RoadRunner/actions/workflows/cmake-Ubuntu-2404.yml/badge.svg)

# About
RoadRunner is a lightweight yet powerful desktop road network editor. Compared to commercial mapping software, it is intended for casual users and hobbyists, providing a free and easy experience without steep learning curve or $$ investment.

The Windows version is recommanded for better UI performance and stability, while RoadRunner remains a cross-platform project.

[Download RoadRunner v0.9](https://github.com/guotata1996/RoadRunner/releases/tag/v0.9)

[![Basic Tutorial](https://img.youtube.com/vi/tsDGT2ElVuM/0.jpg)](https://www.youtube.com/watch?v=tsDGT2ElVuM)

## OpenDRIVE Compatibility
- Internally, RoadRunner heavily relies on [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/) standard for geometric information bookkeeping.
- The exported .xodr does not 100% comply with ASAM OpenDRIVE standard, nor can RoadRunner load an arbitrary .xodr.
- Nonetheless, the exported map can be correctly loaded in [OpenDRIVE online viewer](https://odrviewer.io/).

## Known issues
- Traffic has been seen stuck at highway exits. I guess it has something to do with the application losing focus, but haven't found a way to reliably reproduce it.
- Vehicle spawning on a large map takes a substantial amount of time, and may cause the program to stop responding on weak systems. Be sure to check out the progress bar from the console.

If you find anything bothering or can't get around with, you are welcome to report via Github issues. Please attach the .xodr map and .dat action record if possible.

# Dev environment setup

## Prerequisites
Recommanded OS: Windows 10/11 | Ubuntu 24.04 LTS
- CMake
- cxx build tools ([VS2019 community](https://www.techspot.com/downloads/7241-visual-studio-2019.html) | `sudo apt-get install build-essential gdb`)
- qt5 (`vcpkg install qt5` | `sudo apt install qtbase5-dev`)
- spdlog (`vcpkg install spdlog` | `sudo apt install libspdlog-dev`)
- CGAL ([windows](https://doc.cgal.org/5.6.1/Manual/windows.html) | `sudo apt-get install libcgal-dev`)
> On Ubuntu 22.04 and before, you may need to [manually install](https://doc.cgal.org/5.6.1/Manual/installation.html) CGAL version 5.6.1 for compatibility.

## Clone
```
git clone https://github.com/guotata1996/RoadRunner.git
```

### Windows
- Follow [CGAL Configuring example](https://doc.cgal.org/5.6.1/Manual/windows.html) to configure CMake
- Open the sln with Visual Studio
- Set RoadRunner as Startup Project

### Ubuntu
```
cd RoadRunner
git submodule init && git submodule update
mkdir build && cd build
cmake [--DCMAKE_BUILD_TYPE=Release] ..
make -j2
cpack -G DEB
```


# Notice  
Project Name: RoadRunner

Author: github.com/guotata1996

Copyright (c) 2024-2025 guotata1996

This software is licensed under the Apache License, Version 2.0 (the "License");  
you may not use this file except in compliance with the License.  
You may obtain a copy of the License at:  

`http://www.apache.org/licenses/LICENSE-2.0` 

This project includes source code from other open-source projects:  
- Portions of this software are derived from **libOpenDRIVE** under the Apache 2.0 License.
- Portions of this software are derived from **OpenGLWithQt-Tutorial** under the BSD-3-Clause License.  

See [LICENSE-BSD](https://github.com/ghorwin/OpenGLWithQt-Tutorial/blob/master/LICENSE) and [LICENSE-Apache](https://github.com/pageldev/libOpenDRIVE/blob/main/LICENSE) for the full license texts of these external components.