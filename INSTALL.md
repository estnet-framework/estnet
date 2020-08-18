# OMNeT++ Installation
- Download the latest version of OMNeT++ from https://omnetpp.org/

* **Note:** Before compiling OMNeT++ set the following options in the file configure.user in the downloaded OMNeT++ folder:
  * Set PREFER_CLANG=no
  * Set PREFER_SQLITE_RESULT_FILES=yes in case you prefer sqlite formatted results (recommended)
  

- Follow the **[installation instructions of OMNeT++](https://omnetpp.org/doc/omnetpp/InstallGuide.pdf)**

# INET Installation

- Download [INET 4.2](https://github.com/inet-framework/inet/releases/download/v4.2.0/inet-4.2.0-src.tgz) and rename the folder from inet4 to inet

- Import the project into your OMNeT++ workspace: File -> Import -> Import existing project into ... -> Select the INET folder

# ESTNeT Installation

- Download ESTNet from this repository

- Import the ESTNeT project into your OMNeT++ workspace

# Running a simulation

- Download the template project from https://github.com/estnet-framework/estnet-template

- Import the template project into your OMNeT++ workspace

- Right click on project folder -> Select "Build Project"

- Open the folder "simulations" in the Project Explorer, right click on the file omnetpp.ini and select Run as OMNeT++ Simulation

## trouble-shooting under ubuntu 18.04.2
- make sure not to use openjdk-11-jre, version 8 is ok
- do not forget to install `openscenegraph-plugin-osgearth`
- if the Earth's surface texture is not rendered in 3D scene view, [start omnet++ with](https://old.reddit.com/r/archlinux/comments/8stpmt/how_can_i_get_opengl_33_with_glsl_33_support/e128v1f/)
```
MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 omnetpp
```
