# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_SOURCE_DIR = /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build

# Include any dependencies generated for this target.
include CMakeFiles/bundle_adjust.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/bundle_adjust.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/bundle_adjust.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bundle_adjust.dir/flags.make

CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o: CMakeFiles/bundle_adjust.dir/flags.make
CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o: /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/bundle_adjust.cc
CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o: CMakeFiles/bundle_adjust.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o -MF CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o.d -o CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o -c /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/bundle_adjust.cc

CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/bundle_adjust.cc > CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.i

CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/bundle_adjust.cc -o CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.s

# Object files for target bundle_adjust
bundle_adjust_OBJECTS = \
"CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o"

# External object files for target bundle_adjust
bundle_adjust_EXTERNAL_OBJECTS =

bundle_adjust: CMakeFiles/bundle_adjust.dir/bundle_adjust.cc.o
bundle_adjust: CMakeFiles/bundle_adjust.dir/build.make
bundle_adjust: /usr/local/lib/libceres.a
bundle_adjust: /usr/local/lib/libglog.so
bundle_adjust: /usr/local/lib/libglog.so.0.7.0
bundle_adjust: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
bundle_adjust: /usr/local/lib/libspqr.so
bundle_adjust: /usr/local/lib/libcholmod.so
bundle_adjust: /usr/local/lib/libamd.so
bundle_adjust: /usr/local/lib/libcamd.so
bundle_adjust: /usr/local/lib/libccolamd.so
bundle_adjust: /usr/local/lib/libcolamd.so
bundle_adjust: /usr/local/lib/libsuitesparseconfig.so
bundle_adjust: /usr/local/lib/libopenblas.so
bundle_adjust: CMakeFiles/bundle_adjust.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bundle_adjust"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bundle_adjust.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bundle_adjust.dir/build: bundle_adjust
.PHONY : CMakeFiles/bundle_adjust.dir/build

CMakeFiles/bundle_adjust.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bundle_adjust.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bundle_adjust.dir/clean

CMakeFiles/bundle_adjust.dir/depend:
	cd /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build /home/jackbuntu/workspace/3D-Rig-Example/bundle_adjustment/build/CMakeFiles/bundle_adjust.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/bundle_adjust.dir/depend

