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
CMAKE_SOURCE_DIR = /home/pioneer/group41/src/hippo2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pioneer/group41/build_isolated/hippo2

# Include any dependencies generated for this target.
include CMakeFiles/server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/server.dir/flags.make

CMakeFiles/server.dir/src/server.cpp.o: CMakeFiles/server.dir/flags.make
CMakeFiles/server.dir/src/server.cpp.o: /home/pioneer/group41/src/hippo2/src/server.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pioneer/group41/build_isolated/hippo2/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/server.dir/src/server.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/server.dir/src/server.cpp.o -c /home/pioneer/group41/src/hippo2/src/server.cpp

CMakeFiles/server.dir/src/server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/server.dir/src/server.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pioneer/group41/src/hippo2/src/server.cpp > CMakeFiles/server.dir/src/server.cpp.i

CMakeFiles/server.dir/src/server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/server.dir/src/server.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pioneer/group41/src/hippo2/src/server.cpp -o CMakeFiles/server.dir/src/server.cpp.s

CMakeFiles/server.dir/src/server.cpp.o.requires:
.PHONY : CMakeFiles/server.dir/src/server.cpp.o.requires

CMakeFiles/server.dir/src/server.cpp.o.provides: CMakeFiles/server.dir/src/server.cpp.o.requires
	$(MAKE) -f CMakeFiles/server.dir/build.make CMakeFiles/server.dir/src/server.cpp.o.provides.build
.PHONY : CMakeFiles/server.dir/src/server.cpp.o.provides

CMakeFiles/server.dir/src/server.cpp.o.provides.build: CMakeFiles/server.dir/src/server.cpp.o

# Object files for target server
server_OBJECTS = \
"CMakeFiles/server.dir/src/server.cpp.o"

# External object files for target server
server_EXTERNAL_OBJECTS =

/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: CMakeFiles/server.dir/src/server.cpp.o
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libtf.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libtf2_ros.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libmessage_filters.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libtf2.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libactionlib.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libroscpp.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_signals-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_filesystem-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libcv_bridge.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/librosconsole.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/liblog4cxx.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_regex-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libimage_geometry.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/librostime.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_date_time-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_system-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/libboost_thread-mt.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libcpp_common.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: /opt/ros/hydro/lib/libconsole_bridge.so
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: CMakeFiles/server.dir/build.make
/home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server: CMakeFiles/server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/server.dir/build: /home/pioneer/group41/devel_isolated/hippo2/lib/hippo2/server
.PHONY : CMakeFiles/server.dir/build

CMakeFiles/server.dir/requires: CMakeFiles/server.dir/src/server.cpp.o.requires
.PHONY : CMakeFiles/server.dir/requires

CMakeFiles/server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/server.dir/clean

CMakeFiles/server.dir/depend:
	cd /home/pioneer/group41/build_isolated/hippo2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pioneer/group41/src/hippo2 /home/pioneer/group41/src/hippo2 /home/pioneer/group41/build_isolated/hippo2 /home/pioneer/group41/build_isolated/hippo2 /home/pioneer/group41/build_isolated/hippo2/CMakeFiles/server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/server.dir/depend

