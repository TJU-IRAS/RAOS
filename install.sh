####==================================================
#### Compile, Setup and install script of RAOS
####                For Linux Ubuntu
####                tao.jing @TJU-IRAS
####==================================================

#!/bin/sh

# Get platform type
if [ "$(uname)" = "Darwin" ]; then
    # Do something under Mac OS X platform
    SYSTEM="APPLE"
elif [ "$(expr substr $(uname -s) 1 5)" = "Linux" ]; then
    # Do something under GNU/Linux platform
    SYSTEM="LINUX"
elif [ "$(expr substr $(uname -s) 1 10)" = "MINGW32_NT" ]; then
    # Do something under Windows NT platform
    SYSTEM="WIN32"
fi

# Get the absolute TOP path of this project
prjtop=$(cd "$(dirname "$0")"; pwd)
echo "Absolute path of project top directory is: "$prjtop
sleep 1


##======== Compile 3rd Party softwares ========
echo "Start Compiling 3d party soft ..."
##Uncompress fltk
echo "Uncompress fltk..."
sleep 1
cd $prjtop/3rdparty/
tar -xvf fltk.tar.gz fltk-1.3.3/ >> /dev/null

##===== Compile SOIL =====
echo "Start Compiling SOIL..."
sleep 1
cd $prjtop/3rdparty/SOIL/projects/makefile
mkdir obj
make

##===== Compile glfw =====
echo "Start Compiling glfw..."
sleep 1
cd $prjtop/3rdparty/
tar -xvf glfw.tar.gz glfw-3.3/ >> /dev/null
cd glfw-3.3/
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make

##===== Compile and install glew =====
echo "Start Compiling glew..."
sleep 1
cd $prjtop/3rdparty/
tar -xvf glew.tgz glew-2.1.0/ >> /dev/null
cd glew-2.1.0/
export GLEW_DEST=./install
echo " ==== ${GLEW_DEST} ===="
make
make install

echo "RAOS install completed..."
