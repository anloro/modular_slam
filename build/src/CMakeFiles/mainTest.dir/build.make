# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/angel/Documents/02_master_thesis/modularslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/angel/Documents/02_master_thesis/modularslam/build

# Include any dependencies generated for this target.
include src/CMakeFiles/mainTest.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/mainTest.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/mainTest.dir/flags.make

src/CMakeFiles/mainTest.dir/main.cpp.o: src/CMakeFiles/mainTest.dir/flags.make
src/CMakeFiles/mainTest.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/angel/Documents/02_master_thesis/modularslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/mainTest.dir/main.cpp.o"
	cd /home/angel/Documents/02_master_thesis/modularslam/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mainTest.dir/main.cpp.o -c /home/angel/Documents/02_master_thesis/modularslam/src/main.cpp

src/CMakeFiles/mainTest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mainTest.dir/main.cpp.i"
	cd /home/angel/Documents/02_master_thesis/modularslam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/angel/Documents/02_master_thesis/modularslam/src/main.cpp > CMakeFiles/mainTest.dir/main.cpp.i

src/CMakeFiles/mainTest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mainTest.dir/main.cpp.s"
	cd /home/angel/Documents/02_master_thesis/modularslam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/angel/Documents/02_master_thesis/modularslam/src/main.cpp -o CMakeFiles/mainTest.dir/main.cpp.s

# Object files for target mainTest
mainTest_OBJECTS = \
"CMakeFiles/mainTest.dir/main.cpp.o"

# External object files for target mainTest
mainTest_EXTERNAL_OBJECTS =

src/mainTest: src/CMakeFiles/mainTest.dir/main.cpp.o
src/mainTest: src/CMakeFiles/mainTest.dir/build.make
src/mainTest: src/libmodularslam.a
src/mainTest: src/CMakeFiles/mainTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/angel/Documents/02_master_thesis/modularslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mainTest"
	cd /home/angel/Documents/02_master_thesis/modularslam/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mainTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/mainTest.dir/build: src/mainTest

.PHONY : src/CMakeFiles/mainTest.dir/build

src/CMakeFiles/mainTest.dir/clean:
	cd /home/angel/Documents/02_master_thesis/modularslam/build/src && $(CMAKE_COMMAND) -P CMakeFiles/mainTest.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/mainTest.dir/clean

src/CMakeFiles/mainTest.dir/depend:
	cd /home/angel/Documents/02_master_thesis/modularslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/angel/Documents/02_master_thesis/modularslam /home/angel/Documents/02_master_thesis/modularslam/src /home/angel/Documents/02_master_thesis/modularslam/build /home/angel/Documents/02_master_thesis/modularslam/build/src /home/angel/Documents/02_master_thesis/modularslam/build/src/CMakeFiles/mainTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/mainTest.dir/depend

