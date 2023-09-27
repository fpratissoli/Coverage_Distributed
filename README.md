# Distributed Voronoi Coverage Computation
Discover our C++ library for coverage control. It offers central and distributed approaches, adaptable to your needs. Utilizing Voronoi partitioning, it optimizes spatial coverage across various applications.

---

## How the app works ##
The software shows the convergence of the fully distributed coverage based control inside a predefined environment area.
Once the app is executed, press:

* keyboard key **C**: move the points (robots) to the next step.
* keyboard key **num 0** ... **num 6**: to see the point of view of each robot (from robot 0 to robot 6, for 7 robots in the environment). The robot sensing range of each robot is shown with the detected neighbors. The Voronoi diagram is computed in the sensing area among the detected neighbors.
* keyboard key **R**: return to the global view of the environment, with the global voronoi partitioning of the environment. This global Voronoi computation is only for visualization and debug purposes. The control algorithm is based on the fully distributed approach, with the local voronoi diagram computation.
* keyboard key **G**: if the app is executing a coverage based control with gaussian density function, it is possible to ON/OFF the variance circles and mean position of the gaussian function.
* keyboard key **V**: the computed centroid position of the Voronoi cell of each robot is shown.

---

## Ubuntu Installation and Compilation
Installation guide for Ubuntu code execution

### g++ compiler (tested for version 9.3.0 UBUNTU 20):
    $ sudo apt-get update
    $ sudo apt-get install g++

### SFML library (for further information: https://www.sfml-dev.org/tutorials/2.5/start-linux.php):
    $ sudo apt-get install libsfml-dev

### Compile the code and generate the executable file, in the directory of the main file code:
    # (using Makefile, the executable file will be generated in a build folder)
    $ make      # (or $ make build_main)

    or manually:

    $ g++ -c main.cpp
    $ g++ main.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system 

--- 

## Windows Installation and Compilation
Installation guide for Windows code execution
To execute the algorithm and visualize Voronoi Diagrams you need an IDE and the graphics interface SFML  

We used Code::Blocks IDE (Tested on windows 10)

### Install and configure SFML for Code::Blocks IDE
#### Code::Blocks IDE installation
    First of all you need to install Code::Blocks IDE 20.03
    -go to the download section of the official website --> http://www.codeblocks.org/downloads/26
    -download "codeblocks-20.03mingw-setup.exe"
    -launch "codeblocks-20.03mingw-setup.exe"
    -follow the setup steps to install Code::Blocks IDE (recommended default install)

#### SFML Installation
    Now you can install SFML
    -go to the download section of SFML official website --> https://www.sfml-dev.org/download/sfml/2.5.1/
    -search for "GCC 5.1.0 TDM (SJLJ) - Code::Blocks - 32-bit" and download it
    -extract "GCC 5.1.0 TDM (SJLJ) - Code::Blocks - 32-bit" whetever you want (for example on your C:\)
    The extracted directory "C:\SFML-2.5.1" contains the SFML library files you need so keep in mind where it is.

#### mingw32 Compiler Installation (TDM-GCC-5.1.0-3.exe)
    The compiler versions have to match 100%! So you have to install the mingw32 compiler requested by SFML:
    -download "TDM-GCC-5.1.0-3.exe" from https://sourceforge.net/projects/tdm-gcc/files/TDM-GCC%20Installer/tdm-gcc-5.1.0-3.exe/download
    -launch "TDM-GCC-5.1.0-3.exe"
    -unflag "Check for updated files in the TDM-GCC server" then select 'Create'
    -choose the installation directory (such as C:\) and keep the recommended installation properties to correctly install the compiler
    The compiler's directory "C:\TDM-GCC-32" contains the files you need so keep in mind where it is.


### How to configure your IDE and link SFML library files
#### Code::Blocks IDE Configuration
    -open Code::Blocks
    -select Settings->Compiler->Compiler settings and make sure to flag "Have g++ follow the C++14 ISO C++ language standard [-std=c++14]"
    -select Settings->Compiler->Toolchain executables, choose "..." and set "TDM-GCC-32" as Code::Blocks Compiler's Installation Directory
    Then set the following files (Program Files tab):
        C compiler: mingw32-gcc.exe
        C++ compiler: mingw32-g++.exe
        Linker for dynamics libs: mingw32-g++.exe
        Linker for static libs: ar.exe (default)
        Debugger: GDB/CDB debugger : Default (default)
        Resource compiler: windres.exe (default)
        Make program: mingw32-make.exe (default)

#### SFML Linking steps
    -select Settings->Compiler->Search directories then, under the Compiler tab, select "add"
    -search and set as directory the following one "C:\SFML-2.5.1\include"

    -select Settings->Compiler->Search directories then, under the Linker tab, select "add"
    -search and set as directory the following one "C:\SFML-2.5.1\lib"

#### Adding SFML Library files to your working directory 
if you have any problem doing this part see --> https://www.sfml-dev.org/tutorials/2.5/start-cb.php)

    -outside Code::Blocks navigate to the following directory "C:\SFML-2.5.1\bin" (or alternatively download it from the git folder "SFML_Utilities")
    -copy all the files inside "C:\SFML-2.5.1\bin" (or the ones you have just downloaded from BitBucket folder "SFML_Utilities")
     (The name of the files are: "openal32.dll", "sfml-audio-2.dll", "sfml-audio-d-2.dll", "sfml-graphics-2.dll", "sfml-graphics-d-2.dll", 
        "sfml-network-2.dll", "sfml-network-d-2.dll", "sfml-system-2.dll", "sfml-system-d-2.dll", "sfml-window-2.dll", "sfml-system-d-2.dll")
    -paste all the previous files inside the project working directory "Voronoi_Fortune_Algorithm_New"
    -return to Code::Blocks
    -select Settings->Compiler->Linker settings, click "add" and write manually the following files in the same order:
        sfml-graphics-d
        sfml-window-d
        sfml-audio-d
        sfml-network-d
        sfml-system-d
        sfml-graphics
        sfml-window
        sfml-audio
        sfml-network
        sfml-system
    Now you should be able to run the program code properly.
    (In the main.cpp file you have to keep "#include <SFML/Graphics.hpp>" and "#include <SFML/OpenGL.hpp>" to run the program code).


Remember to create a new workspace in Code::Blocks and then add the files main.cpp and the header files contained in the project directory you have downloaded in order to visualize the project tree.



