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
include CMakeFiles/state_machine_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/state_machine_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/state_machine_example.dir/flags.make

CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o: CMakeFiles/state_machine_example.dir/flags.make
CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o: ../example/state_machine/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lwx/unitree_sdk2-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o -c /home/lwx/unitree_sdk2-main/example/state_machine/main.cpp

CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lwx/unitree_sdk2-main/example/state_machine/main.cpp > CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.i

CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lwx/unitree_sdk2-main/example/state_machine/main.cpp -o CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.s

# Object files for target state_machine_example
state_machine_example_OBJECTS = \
"CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o"

# External object files for target state_machine_example
state_machine_example_EXTERNAL_OBJECTS =

state_machine_example: CMakeFiles/state_machine_example.dir/example/state_machine/main.cpp.o
state_machine_example: CMakeFiles/state_machine_example.dir/build.make
state_machine_example: CMakeFiles/state_machine_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lwx/unitree_sdk2-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable state_machine_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/state_machine_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/state_machine_example.dir/build: state_machine_example

.PHONY : CMakeFiles/state_machine_example.dir/build

CMakeFiles/state_machine_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/state_machine_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/state_machine_example.dir/clean

CMakeFiles/state_machine_example.dir/depend:
	cd /home/lwx/unitree_sdk2-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwx/unitree_sdk2-main /home/lwx/unitree_sdk2-main /home/lwx/unitree_sdk2-main/build /home/lwx/unitree_sdk2-main/build /home/lwx/unitree_sdk2-main/build/CMakeFiles/state_machine_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/state_machine_example.dir/depend

