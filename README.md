RAOS -- Simulation Framework for Robot Active Olfaction
=======

Copyright @ TJU-IRAS. Only for academic research.

Dependencies:
----
(1) OpenGL library<br>
(2) boost<br>
(3) make, cmake, gcc, g++, gfortran<br>
(4) for rotor wake calculation:<br>
    cuda toolkit, and a graphic card<br>

Note: For the following libs, we provide pre-built lib files in 3rdparty, which may be dependent on your system configuration.

If any of these libs is not working properlly, you can download the source of the library and compile it on your own PC. And also change the cmake file configuration.

(5) glm ([OpenGL Mathematics](http://glm.g-truc.net/)): https://github.com/g-truc/glm, https://sourceforge.net/projects/ogl-math/

Note: For the following libs, the install.sh file will install the environment automatically.

If any problems occurs, you may need to debug the install process according to the prompted infomation.

(6) SOIL (Simple OpenGL Image Library) : http://www.lonesock.net/soil.html

(7) GLFW: https://www.glfw.org/

(8) GLEW (The OpenGL Extension Wrangler Library): http://glew.sourceforge.net/

Install:
----

Run the install.sh to enable some 3rdparty. You may need chmod a+x install.sh.

Compilation:
----

General CMake compilation method (Review some libraries path configuration):

mkdir build & cd build

cmake ..

Help:
----
(1) How to check GPU compute compability ?

Refer to https://developer.nvidia.com/cuda-gpus#compute;

