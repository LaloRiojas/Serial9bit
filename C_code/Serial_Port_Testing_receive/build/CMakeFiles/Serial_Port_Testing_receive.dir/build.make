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
CMAKE_SOURCE_DIR = "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build"

# Include any dependencies generated for this target.
include CMakeFiles/Serial_Port_Testing_receive.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Serial_Port_Testing_receive.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Serial_Port_Testing_receive.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Serial_Port_Testing_receive.dir/flags.make

CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o: CMakeFiles/Serial_Port_Testing_receive.dir/flags.make
CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o: ../main.c
CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o: CMakeFiles/Serial_Port_Testing_receive.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o -MF CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o.d -o CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o -c "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/main.c"

CMakeFiles/Serial_Port_Testing_receive.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Serial_Port_Testing_receive.dir/main.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/main.c" > CMakeFiles/Serial_Port_Testing_receive.dir/main.c.i

CMakeFiles/Serial_Port_Testing_receive.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Serial_Port_Testing_receive.dir/main.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/main.c" -o CMakeFiles/Serial_Port_Testing_receive.dir/main.c.s

# Object files for target Serial_Port_Testing_receive
Serial_Port_Testing_receive_OBJECTS = \
"CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o"

# External object files for target Serial_Port_Testing_receive
Serial_Port_Testing_receive_EXTERNAL_OBJECTS =

Serial_Port_Testing_receive: CMakeFiles/Serial_Port_Testing_receive.dir/main.c.o
Serial_Port_Testing_receive: CMakeFiles/Serial_Port_Testing_receive.dir/build.make
Serial_Port_Testing_receive: CMakeFiles/Serial_Port_Testing_receive.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable Serial_Port_Testing_receive"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Serial_Port_Testing_receive.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Serial_Port_Testing_receive.dir/build: Serial_Port_Testing_receive
.PHONY : CMakeFiles/Serial_Port_Testing_receive.dir/build

CMakeFiles/Serial_Port_Testing_receive.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Serial_Port_Testing_receive.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Serial_Port_Testing_receive.dir/clean

CMakeFiles/Serial_Port_Testing_receive.dir/depend:
	cd "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive" "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive" "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build" "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build" "/home/lalo/linux code serial port/linux_serial_port_testing/C_code/Serial_Port_Testing_receive/build/CMakeFiles/Serial_Port_Testing_receive.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/Serial_Port_Testing_receive.dir/depend

