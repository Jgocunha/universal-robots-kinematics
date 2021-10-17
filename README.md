# universal-robots-kinematics

This work was developed in the context of our MSc dissertations: *A Collaborative Work Cell to Improve Ergonomics and Productivity* by João Cunha, and *Human-Like Motion Generation through Waypoints for Collaborative Robots in Industry 4.0* by João Pereira, in which we got to work with the collaborative robotic arm **UR10 e-series**. 

Before producing our kinematics solution, we conducted a comprehensive literature review on the UR robots’ kinematics (which can be consulted in the *"Articles"* folder of this repository) and realised there was a lack of thorough and detailed analysis of their kinematics. Additionally, we found the [Universal Robots’ documentation](https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/) confusing and unclear. 

Thus, the main goal of this work is to provide an **explicit and transparent guide into the UR robots kinematics** (using by reference the UR10 e-series) by expanding on the literature we found and describing every part of our analysis. 

We present a **forward kinematic solution based on the Modified Denavit-Hartenberg convention** and an **inverse kinematic solution based on a geometric analysis**. The code is available in C++ and MATLAB, both with integration with CoppeliaSim.

<!-- For more information refer to the [Wiki](https://github.com/Jgocunha/universal-robots-kinematics/wiki) page.-->

***

## C++ source code

The solution was built using Microsoft Visual Studio 2019, and C++ 20 standard.

### The `UR class`

...

***

## Dependencies

### Eigen

[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

In the context of this solution it is used to simplify the creation of 2D arrays, and matrix operations.

To use Eigen:
1. Download the source code from their website;
2. Extract the contents of the .rar to a directory of your choice (usually directly to your C:/ drive);
3. On your project properties:
    - C/C++ -> Additional Include Directories -> *your Eigen directory e.g. C:\Eigen*;
    - now you can include `#include <Eigen\Dense>`.

### CoppeliaSim

This isn't necessary if you only want to use the forward and inverse kinematics solutions.
To run the CoppeliaSim integration, you have to install CoppeliaSim. To use their [C++ remote API](https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm):
1. Include the following files in your project directory: extApi.h, extApi.c, extApiPlatform.h, and extApiPlatform.c; which are located in CoppeliaSim's installation directory, under programming/remoteApi;
2. Define `NON_MATLAB_PARSING` and `MAX_EXT_API_CONNECTIONS=255` (and optionally `DO_NOT_USE_SHARED_MEMORY`) as a preprocessor definition;
3. Include additional directories remoteApi and include;
4. If you aren't interested in using the secure version of their calls (like fopen_s), you need to place a definition of `_CRT_SECURE_NO_DEPRECATE` before your included header files.

***

## Benchmarking

Benchmarking consists of:
 1. getting [compute times](https://stackoverflow.com/questions/22387586/measuring-execution-time-of-a-function-in-c) of forward and inverse kinematics functions;
 2. getting the error of the solutions.

The solutions were tested for a set of 100000 randomly generated target tip poses (the scripts were run in a Ryzen 5 3600 CPU at 4.28GHz).

MATLAB's compute times are 10x slower than with C++, so **the C++ solution is obviously recommended for use**.

 ### Average Computation Times (in seconds)

||MATLAB|C++|
|---|---|---|
|Forward Kinematics|1.832571E-05|1.39434e-06|
|Inverse Kinematics|1.612797E-04|1.07403E-05|

### Average error

To compute the average error the following flowchart was followed:

![How to get the errors](.\Resources\errorSolsFlow.png)

||*x*|*y*|*z*|*alpha*|*beta*|*gamma*|
|-|--|---|---|-------|------|-------|
average error|7.45E-14|1.86E-14|7.45E-14|4.14E-06|4.14E-06|4.14E-06|

**Units in**: *x, y, z* metres, *alpha, beta, gamma* radians.
