
<div align="center" id="top"> 
  <img src="./.github/app.gif" alt="Linux_serial_port_testing" />

  &#xa0;

</div>

<h1 align="center">Serial9Bit</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/LaloRiojas/Serial9bit?color=56BEB8">
 <img alt="Github language count" src="https://img.shields.io/github/languages/count/LaloRiojas/Serial9bit?color=56BEB8" >
<img alt="Repository size" src="https://img.shields.io/github/repo-size/LaloRiojas/Serial9bit?color=56BEB8">
<img alt="License" src="https://img.shields.io/github/license/LaloRiojas/Serial9bit?color=56BEB8">
</p>


<h4 align="center"> 
	🚧  Serial9bit 🚀 Under construction...  🚧
</h4> 

<hr> 

<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#rocket-technologies">Technologies</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-requirements">Requirements</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Starting</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/{{YOUR_GITHUB_USERNAME}}" target="_blank">Author</a>
</p>

<br>

## :dart: About ##

This project is a 9-bit library with a command-line interface (CLI). It provides functionality for working with 9-bit data on DB9 Serial ports. The library is designed to Send and recienve 9 bit data over a serial port. The CLI provides a user-friendly interface for executing operations on 9-bit numbers and can be used as a standalone tool or integrated into other applications.

The library is writtten in C and is designed to be used in other projects that require 9-bit data transmission over a serial port. This is common in old serial communication protocols that use the 9th bit as a wakeup bit or for error checking etc. The library provides functions for sending and receiving 9-bit data over a serial port, as well as functions for setting up the serial port. more information can be found in the Serial.h file.

Please note that this project is intended to be used as a library, but it also includes a CLI for testing and demonstration purposes. the CLI is very much in ALPHA stage and is probably not working. no testing has been done on it 


## :sparkles: Features ##

:heavy_check_mark: set up the Serial porst for 9-bit communication; 

:heavy_check_mark: Send 9-bit data over a serial port;

:heavy_check_mark: Receive 9-bit data over a serial port;

:heavy_check_mark: print 9-bit data in a human readable format;


## :rocket: Technologies ##

The following tools were used in this project:

- [C](https://en.wikipedia.org/wiki/C_(programming_language))
- [POSIX](https://en.wikipedia.org/wiki/POSIX)
- [CMAKE](https://cmake.org/)

## :white_check_mark: Requirements ##

Before starting :checkered_flag:, you need to have [Git](https://git-scm.com) and [CMAKE](https://cmake.org/) installed.

## :checkered_flag: Starting ##

```bash
# Clone this project
$ git clone https://github.com/LaloRiojas/linux_serial_port_testing

# Access
$ cd linux_serial_port_testing

# Install dependencies
$ mkdir build && cd build 

# USE CMAKE
$ cmake .. 
$ cmake --build .

# Run the project
$ ./Serial9Bit #command line args

```
## :memo: License ##

NOLICENSE USE AT YOUR OWN RISK AND FOLLOW POSIX GUIDELINES AND OTHER DEPENDENCIES LICENSES


&#xa0;

<a href="#top">Back to top</a>
