# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /home/pc/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/pc/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pc/Documents/GitHub/IW2.0/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pc/Documents/GitHub/IW2.0/build

# Include any dependencies generated for this target.
include zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/compiler_depend.make

# Include the progress variables for this target.
include zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/progress.make

# Include the compile flags for this target's objects.
include zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/flags.make

zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o: zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/flags.make
zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o: /home/pc/Documents/GitHub/IW2.0/src/zed-ros-examples/tutorials/zed_tracking_sub_tutorial/src/zed_tracking_sub_tutorial.cpp
zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o: zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pc/Documents/GitHub/IW2.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o"
	cd /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o -MF CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o.d -o CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o -c /home/pc/Documents/GitHub/IW2.0/src/zed-ros-examples/tutorials/zed_tracking_sub_tutorial/src/zed_tracking_sub_tutorial.cpp

zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.i"
	cd /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pc/Documents/GitHub/IW2.0/src/zed-ros-examples/tutorials/zed_tracking_sub_tutorial/src/zed_tracking_sub_tutorial.cpp > CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.i

zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.s"
	cd /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pc/Documents/GitHub/IW2.0/src/zed-ros-examples/tutorials/zed_tracking_sub_tutorial/src/zed_tracking_sub_tutorial.cpp -o CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.s

# Object files for target zed_tracking_sub
zed_tracking_sub_OBJECTS = \
"CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o"

# External object files for target zed_tracking_sub
zed_tracking_sub_EXTERNAL_OBJECTS =

/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/src/zed_tracking_sub_tutorial.cpp.o
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/build.make
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/libroscpp.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/librosconsole.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/librostime.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /opt/ros/melodic/lib/libcpp_common.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub: zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pc/Documents/GitHub/IW2.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub"
	cd /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zed_tracking_sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/build: /home/pc/Documents/GitHub/IW2.0/devel/lib/zed_tracking_sub_tutorial/zed_tracking_sub
.PHONY : zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/build

zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/clean:
	cd /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial && $(CMAKE_COMMAND) -P CMakeFiles/zed_tracking_sub.dir/cmake_clean.cmake
.PHONY : zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/clean

zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/depend:
	cd /home/pc/Documents/GitHub/IW2.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc/Documents/GitHub/IW2.0/src /home/pc/Documents/GitHub/IW2.0/src/zed-ros-examples/tutorials/zed_tracking_sub_tutorial /home/pc/Documents/GitHub/IW2.0/build /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial /home/pc/Documents/GitHub/IW2.0/build/zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : zed-ros-examples/tutorials/zed_tracking_sub_tutorial/CMakeFiles/zed_tracking_sub.dir/depend

