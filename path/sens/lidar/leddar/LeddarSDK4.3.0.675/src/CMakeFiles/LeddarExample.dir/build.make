# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src

# Include any dependencies generated for this target.
include CMakeFiles/LeddarExample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LeddarExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LeddarExample.dir/flags.make

CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o: CMakeFiles/LeddarExample.dir/flags.make
CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o: LeddarExample/LeddarExample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o -c /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/LeddarExample/LeddarExample.cpp

CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/LeddarExample/LeddarExample.cpp > CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.i

CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/LeddarExample/LeddarExample.cpp -o CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.s

# Object files for target LeddarExample
LeddarExample_OBJECTS = \
"CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o"

# External object files for target LeddarExample
LeddarExample_EXTERNAL_OBJECTS =

LeddarExample: CMakeFiles/LeddarExample.dir/LeddarExample/LeddarExample.cpp.o
LeddarExample: CMakeFiles/LeddarExample.dir/build.make
LeddarExample: libLC4.a
LeddarExample: CMakeFiles/LeddarExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable LeddarExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LeddarExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LeddarExample.dir/build: LeddarExample

.PHONY : CMakeFiles/LeddarExample.dir/build

CMakeFiles/LeddarExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LeddarExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LeddarExample.dir/clean

CMakeFiles/LeddarExample.dir/depend:
	cd /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src /windows/Linux/Volvo_rolys/path/sens/lidar/leddar/LeddarSDK4.3.0.675/src/CMakeFiles/LeddarExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LeddarExample.dir/depend
