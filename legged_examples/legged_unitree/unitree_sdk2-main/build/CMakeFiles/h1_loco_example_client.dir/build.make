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
CMAKE_SOURCE_DIR = /home/lwx/unitree_sdk2-main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lwx/unitree_sdk2-main/build

# Include any dependencies generated for this target.
include CMakeFiles/h1_loco_example_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/h1_loco_example_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/h1_loco_example_client.dir/flags.make

CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o: CMakeFiles/h1_loco_example_client.dir/flags.make
CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o: ../example/humanoid/high_level/h1_loco_example_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lwx/unitree_sdk2-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o -c /home/lwx/unitree_sdk2-main/example/humanoid/high_level/h1_loco_example_client.cpp

CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lwx/unitree_sdk2-main/example/humanoid/high_level/h1_loco_example_client.cpp > CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.i

CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lwx/unitree_sdk2-main/example/humanoid/high_level/h1_loco_example_client.cpp -o CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.s

# Object files for target h1_loco_example_client
h1_loco_example_client_OBJECTS = \
"CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o"

# External object files for target h1_loco_example_client
h1_loco_example_client_EXTERNAL_OBJECTS =

h1_loco_example_client: CMakeFiles/h1_loco_example_client.dir/example/humanoid/high_level/h1_loco_example_client.cpp.o
h1_loco_example_client: CMakeFiles/h1_loco_example_client.dir/build.make
h1_loco_example_client: CMakeFiles/h1_loco_example_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lwx/unitree_sdk2-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable h1_loco_example_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/h1_loco_example_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/h1_loco_example_client.dir/build: h1_loco_example_client

.PHONY : CMakeFiles/h1_loco_example_client.dir/build

CMakeFiles/h1_loco_example_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/h1_loco_example_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/h1_loco_example_client.dir/clean

CMakeFiles/h1_loco_example_client.dir/depend:
	cd /home/lwx/unitree_sdk2-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwx/unitree_sdk2-main /home/lwx/unitree_sdk2-main /home/lwx/unitree_sdk2-main/build /home/lwx/unitree_sdk2-main/build /home/lwx/unitree_sdk2-main/build/CMakeFiles/h1_loco_example_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/h1_loco_example_client.dir/depend

