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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/cspc_lidar.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cspc_lidar.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cspc_lidar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cspc_lidar.dir/flags.make

CMakeFiles/cspc_lidar.dir/calibration.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/calibration.cpp.o: ../calibration.cpp
CMakeFiles/cspc_lidar.dir/calibration.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cspc_lidar.dir/calibration.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/calibration.cpp.o -MF CMakeFiles/cspc_lidar.dir/calibration.cpp.o.d -o CMakeFiles/cspc_lidar.dir/calibration.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/calibration.cpp

CMakeFiles/cspc_lidar.dir/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/calibration.cpp > CMakeFiles/cspc_lidar.dir/calibration.cpp.i

CMakeFiles/cspc_lidar.dir/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/calibration.cpp -o CMakeFiles/cspc_lidar.dir/calibration.cpp.s

CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o: ../lidar_data_processing.cpp
CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o -MF CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o.d -o CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_data_processing.cpp

CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_data_processing.cpp > CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.i

CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_data_processing.cpp -o CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.s

CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o: ../lidar_information.cpp
CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o -MF CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o.d -o CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_information.cpp

CMakeFiles/cspc_lidar.dir/lidar_information.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/lidar_information.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_information.cpp > CMakeFiles/cspc_lidar.dir/lidar_information.cpp.i

CMakeFiles/cspc_lidar.dir/lidar_information.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/lidar_information.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/lidar_information.cpp -o CMakeFiles/cspc_lidar.dir/lidar_information.cpp.s

CMakeFiles/cspc_lidar.dir/main.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/main.cpp.o: ../main.cpp
CMakeFiles/cspc_lidar.dir/main.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/cspc_lidar.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/main.cpp.o -MF CMakeFiles/cspc_lidar.dir/main.cpp.o.d -o CMakeFiles/cspc_lidar.dir/main.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/main.cpp

CMakeFiles/cspc_lidar.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/main.cpp > CMakeFiles/cspc_lidar.dir/main.cpp.i

CMakeFiles/cspc_lidar.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/main.cpp -o CMakeFiles/cspc_lidar.dir/main.cpp.s

CMakeFiles/cspc_lidar.dir/mtime.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/mtime.cpp.o: ../mtime.cpp
CMakeFiles/cspc_lidar.dir/mtime.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/cspc_lidar.dir/mtime.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/mtime.cpp.o -MF CMakeFiles/cspc_lidar.dir/mtime.cpp.o.d -o CMakeFiles/cspc_lidar.dir/mtime.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/mtime.cpp

CMakeFiles/cspc_lidar.dir/mtime.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/mtime.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/mtime.cpp > CMakeFiles/cspc_lidar.dir/mtime.cpp.i

CMakeFiles/cspc_lidar.dir/mtime.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/mtime.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/mtime.cpp -o CMakeFiles/cspc_lidar.dir/mtime.cpp.s

CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o: ../node_lidar.cpp
CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o -MF CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o.d -o CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/node_lidar.cpp

CMakeFiles/cspc_lidar.dir/node_lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/node_lidar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/node_lidar.cpp > CMakeFiles/cspc_lidar.dir/node_lidar.cpp.i

CMakeFiles/cspc_lidar.dir/node_lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/node_lidar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/node_lidar.cpp -o CMakeFiles/cspc_lidar.dir/node_lidar.cpp.s

CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o: ../point_cloud_optimize.cpp
CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o -MF CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o.d -o CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/point_cloud_optimize.cpp

CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/point_cloud_optimize.cpp > CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.i

CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/point_cloud_optimize.cpp -o CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.s

CMakeFiles/cspc_lidar.dir/serial_port.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/serial_port.cpp.o: ../serial_port.cpp
CMakeFiles/cspc_lidar.dir/serial_port.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/cspc_lidar.dir/serial_port.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/serial_port.cpp.o -MF CMakeFiles/cspc_lidar.dir/serial_port.cpp.o.d -o CMakeFiles/cspc_lidar.dir/serial_port.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/serial_port.cpp

CMakeFiles/cspc_lidar.dir/serial_port.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/serial_port.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/serial_port.cpp > CMakeFiles/cspc_lidar.dir/serial_port.cpp.i

CMakeFiles/cspc_lidar.dir/serial_port.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/serial_port.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/serial_port.cpp -o CMakeFiles/cspc_lidar.dir/serial_port.cpp.s

CMakeFiles/cspc_lidar.dir/timer.cpp.o: CMakeFiles/cspc_lidar.dir/flags.make
CMakeFiles/cspc_lidar.dir/timer.cpp.o: ../timer.cpp
CMakeFiles/cspc_lidar.dir/timer.cpp.o: CMakeFiles/cspc_lidar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/cspc_lidar.dir/timer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cspc_lidar.dir/timer.cpp.o -MF CMakeFiles/cspc_lidar.dir/timer.cpp.o.d -o CMakeFiles/cspc_lidar.dir/timer.cpp.o -c /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/timer.cpp

CMakeFiles/cspc_lidar.dir/timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cspc_lidar.dir/timer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/timer.cpp > CMakeFiles/cspc_lidar.dir/timer.cpp.i

CMakeFiles/cspc_lidar.dir/timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cspc_lidar.dir/timer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/timer.cpp -o CMakeFiles/cspc_lidar.dir/timer.cpp.s

# Object files for target cspc_lidar
cspc_lidar_OBJECTS = \
"CMakeFiles/cspc_lidar.dir/calibration.cpp.o" \
"CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o" \
"CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o" \
"CMakeFiles/cspc_lidar.dir/main.cpp.o" \
"CMakeFiles/cspc_lidar.dir/mtime.cpp.o" \
"CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o" \
"CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o" \
"CMakeFiles/cspc_lidar.dir/serial_port.cpp.o" \
"CMakeFiles/cspc_lidar.dir/timer.cpp.o"

# External object files for target cspc_lidar
cspc_lidar_EXTERNAL_OBJECTS =

cspc_lidar: CMakeFiles/cspc_lidar.dir/calibration.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/lidar_data_processing.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/lidar_information.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/main.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/mtime.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/node_lidar.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/point_cloud_optimize.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/serial_port.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/timer.cpp.o
cspc_lidar: CMakeFiles/cspc_lidar.dir/build.make
cspc_lidar: CMakeFiles/cspc_lidar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable cspc_lidar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cspc_lidar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cspc_lidar.dir/build: cspc_lidar
.PHONY : CMakeFiles/cspc_lidar.dir/build

CMakeFiles/cspc_lidar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cspc_lidar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cspc_lidar.dir/clean

CMakeFiles/cspc_lidar.dir/depend:
	cd /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build /home/pedro/dev_ws/src/cspc_lidar_sdk_ros2/sdk/build/CMakeFiles/cspc_lidar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cspc_lidar.dir/depend
