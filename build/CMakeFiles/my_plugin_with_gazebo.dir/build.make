# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/viki/gaia

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/viki/gaia/build

# Include any dependencies generated for this target.
include CMakeFiles/my_plugin_with_gazebo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_plugin_with_gazebo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_plugin_with_gazebo.dir/flags.make

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o: CMakeFiles/my_plugin_with_gazebo.dir/flags.make
CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o: ../my_plugin_with_gazebo.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/viki/gaia/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o -c /home/viki/gaia/my_plugin_with_gazebo.cc

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/viki/gaia/my_plugin_with_gazebo.cc > CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.i

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/viki/gaia/my_plugin_with_gazebo.cc -o CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.s

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.requires:
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.requires

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.provides: CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.requires
	$(MAKE) -f CMakeFiles/my_plugin_with_gazebo.dir/build.make CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.provides.build
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.provides

CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.provides.build: CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o

# Object files for target my_plugin_with_gazebo
my_plugin_with_gazebo_OBJECTS = \
"CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o"

# External object files for target my_plugin_with_gazebo
my_plugin_with_gazebo_EXTERNAL_OBJECTS =

libmy_plugin_with_gazebo.so: CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o
libmy_plugin_with_gazebo.so: CMakeFiles/my_plugin_with_gazebo.dir/build.make
libmy_plugin_with_gazebo.so: CMakeFiles/my_plugin_with_gazebo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libmy_plugin_with_gazebo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_plugin_with_gazebo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_plugin_with_gazebo.dir/build: libmy_plugin_with_gazebo.so
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/build

CMakeFiles/my_plugin_with_gazebo.dir/requires: CMakeFiles/my_plugin_with_gazebo.dir/my_plugin_with_gazebo.cc.o.requires
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/requires

CMakeFiles/my_plugin_with_gazebo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_plugin_with_gazebo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/clean

CMakeFiles/my_plugin_with_gazebo.dir/depend:
	cd /home/viki/gaia/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/viki/gaia /home/viki/gaia /home/viki/gaia/build /home/viki/gaia/build /home/viki/gaia/build/CMakeFiles/my_plugin_with_gazebo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_plugin_with_gazebo.dir/depend

