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
CMAKE_SOURCE_DIR = D:\Graphics\compile_flags

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Graphics\compile_flags\build

# Include any dependencies generated for this target.
include CMakeFiles/cmake_examples_compile_flags.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cmake_examples_compile_flags.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cmake_examples_compile_flags.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmake_examples_compile_flags.dir/flags.make

CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj: CMakeFiles/cmake_examples_compile_flags.dir/flags.make
CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj: D:/Graphics/compile_flags/main.cpp
CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj: CMakeFiles/cmake_examples_compile_flags.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Graphics\compile_flags\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj -MF CMakeFiles\cmake_examples_compile_flags.dir\main.cpp.obj.d -o CMakeFiles\cmake_examples_compile_flags.dir\main.cpp.obj -c D:\Graphics\compile_flags\main.cpp

CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.i"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Graphics\compile_flags\main.cpp > CMakeFiles\cmake_examples_compile_flags.dir\main.cpp.i

CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.s"
	D:\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Graphics\compile_flags\main.cpp -o CMakeFiles\cmake_examples_compile_flags.dir\main.cpp.s

# Object files for target cmake_examples_compile_flags
cmake_examples_compile_flags_OBJECTS = \
"CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj"

# External object files for target cmake_examples_compile_flags
cmake_examples_compile_flags_EXTERNAL_OBJECTS =

cmake_examples_compile_flags.exe: CMakeFiles/cmake_examples_compile_flags.dir/main.cpp.obj
cmake_examples_compile_flags.exe: CMakeFiles/cmake_examples_compile_flags.dir/build.make
cmake_examples_compile_flags.exe: CMakeFiles/cmake_examples_compile_flags.dir/linkLibs.rsp
cmake_examples_compile_flags.exe: CMakeFiles/cmake_examples_compile_flags.dir/objects1.rsp
cmake_examples_compile_flags.exe: CMakeFiles/cmake_examples_compile_flags.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Graphics\compile_flags\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cmake_examples_compile_flags.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\cmake_examples_compile_flags.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmake_examples_compile_flags.dir/build: cmake_examples_compile_flags.exe
.PHONY : CMakeFiles/cmake_examples_compile_flags.dir/build

CMakeFiles/cmake_examples_compile_flags.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\cmake_examples_compile_flags.dir\cmake_clean.cmake
.PHONY : CMakeFiles/cmake_examples_compile_flags.dir/clean

CMakeFiles/cmake_examples_compile_flags.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Graphics\compile_flags D:\Graphics\compile_flags D:\Graphics\compile_flags\build D:\Graphics\compile_flags\build D:\Graphics\compile_flags\build\CMakeFiles\cmake_examples_compile_flags.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/cmake_examples_compile_flags.dir/depend

