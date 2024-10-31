cmake_minimum_required(VERSION 2.8)
project( orthus )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Build the core library
file(GLOB SOURCES
    src/*.h
    src/*.cpp
)
add_executable( orthus ${SOURCES} )
target_link_libraries( orthus ${OpenCV_LIBS})