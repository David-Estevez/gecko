GECKO
=====

Gesture Control via webcam

Authors:

 *  [David Estevez Fernandez](https://github.com/David-Estevez)
 *  [Irene Sanz Nieto](https://github.com/irenesanznieto)

<img src="https://raw.github.com/David-Estevez/gecko/master/doc/images/hand1.jpg" alt="Screenshot" style="width: 500px;" />


Index
-------------------------------------------------------------------
 * 1.Introduction
 * 2.Compile and run
     * 2.1. Dependencies
     * 2.2. Compiling
     * 2.3. Doxygen documentation
     * 2.4. More info


1. Introduction
---------------------------------------------------------------------
GECKO is a gesture recognition software that allows the user a basic control of the computer's cursor and click functionalities, as well as launching user-defined commands.


2. Compile and run
---------------------------------------------------------------------
###2.1. Dependencies###
This software has currently support only for GNU/Linux systems, as the cursor control is platform-dependent, but as it has a modular design it can be easily extended for being used on Windows or Mac platforms. Nevertheless we do not have any early plans of porting it to other platforms.

The requirements for compiling and using the software are:

 * [CMake](http://www.cmake.org/) (minimum version 2.8)
 * [OpenCV](http://opencv.org/) (v2.4.6.1).
 *  X11 (GNU/Linux libraries that allow the interface with the X server for controlling the cursor).

###2.2 Compiling###
#####2.2.1. Using CMake#####
The folder structure used is the typical of a CMake project. In order to compile the project open a terminal in the build directory (gecko/build) and run this commands:

        $ cmake ..
        $ make

#####2.2.1. Using QtCreator#####
To open the software as a QtCreator project, the only thing needed is to open the main CMakeLists.txt (gecko/CMakeLists.txt) with QtCreator. This will parse the whole project.
Afterwards, press the "build" icon to build the project.

3. Doxygen documentacion
--------------------------------------------------------------------

There is doxygen-generated documentation available for GECKO [here](http://david-estevez.github.io/gecko).

4. More info
--------------------------------------------------------------------
