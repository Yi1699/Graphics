# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.27

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Graphics\optimizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Graphics\optimizer\build

# Include any dependencies generated for this target.
include CMakeFiles/optimizer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/optimizer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/optimizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optimizer.dir/flags.make

CMakeFiles/optimizer.dir/main.cpp.obj: CMakeFiles/optimizer.dir/flags.make
CMakeFiles/optimizer.dir/main.cpp.obj: CMakeFiles/optimizer.dir/includes_CXX.rsp
CMakeFiles/optimizer.dir/main.cpp.obj: D:/Graphics/optimizer/main.cpp
CMakeFiles/optimizer.dir/main.cpp.obj: CMakeFiles/optimizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Graphics\optimizer\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optimizer.dir/main.cpp.obj"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/optimizer.dir/main.cpp.obj -MF CMakeFiles\optimizer.dir\main.cpp.obj.d -o CMakeFiles\optimizer.dir\main.cpp.obj -c D:\Graphics\optimizer\main.cpp

CMakeFiles/optimizer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/optimizer.dir/main.cpp.i"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Graphics\optimizer\main.cpp > CMakeFiles\optimizer.dir\main.cpp.i

CMakeFiles/optimizer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/optimizer.dir/main.cpp.s"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Graphics\optimizer\main.cpp -o CMakeFiles\optimizer.dir\main.cpp.s

# Object files for target optimizer
optimizer_OBJECTS = \
"CMakeFiles/optimizer.dir/main.cpp.obj"

# External object files for target optimizer
optimizer_EXTERNAL_OBJECTS =

optimizer.exe: CMakeFiles/optimizer.dir/main.cpp.obj
optimizer.exe: CMakeFiles/optimizer.dir/build.make
optimizer.exe: CMakeFiles/optimizer.dir/linkLibs.rsp
optimizer.exe: CMakeFiles/optimizer.dir/objects1.rsp
optimizer.exe: CMakeFiles/optimizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Graphics\optimizer\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable optimizer.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\optimizer.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optimizer.dir/build: optimizer.exe
.PHONY : CMakeFiles/optimizer.dir/build

CMakeFiles/optimizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\optimizer.dir\cmake_clean.cmake
.PHONY : CMakeFiles/optimizer.dir/clean

CMakeFiles/optimizer.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Graphics\optimizer D:\Graphics\optimizer D:\Graphics\optimizer\build D:\Graphics\optimizer\build D:\Graphics\optimizer\build\CMakeFiles\optimizer.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/optimizer.dir/depend

