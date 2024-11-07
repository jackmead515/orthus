cmake_minimum_required(VERSION 2.8)
project( orthus )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Build for release (with optimizations) by default
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Build the core library
file(GLOB SOURCES
    src/*.h
    src/*.cpp
)
add_executable( orthus ${SOURCES} )
target_link_libraries( orthus ${OpenCV_LIBS})