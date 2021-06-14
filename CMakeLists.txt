# Set minimum version of CMake.
cmake_minimum_required(VERSION 3.0)

# Set project name and version
project(modularslam VERSION 1.2.3)

# Library sources
set(LIBRARY_FOLDER 
    src/
    )
    
# Set environment variables
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)


add_subdirectory(${LIBRARY_FOLDER})


