##===============================================================
# This file installs the dependencies which are required
# Note: This file needs to be executed with ROOT previlege
#
# Author: Roice (LUO Bing)
# Date:   2014-11-25 Create this file
#	  2017-07-02 Modify this file to adapt Debian Stretch (9.0)

#!/bin/sh

#======== obtaining the Build Tools required ========
# for Debian based Dist, tested on Jessie
#-------- Build Tools --------
# GNU Make
apt-get install make
# AWK, sed, and other Unix utilities, shipped with any modern Unix system
#apt-get install gawk
# Bison
apt-get install bison
# Flex
apt-get install flex
# Autoconf
apt-get install autoconf
# Automake
apt-get install automake
# Libtool
apt-get install libtool
# Cmake
apt-get install cmake

#======== obtaining the dependencies OCTAVE require ========
#-------- Build Dependencies --------
# for Debian based LINUX dist, tested on Jessie
apt-get build-dep octave
#-------- External Packages --------
# BLAS
apt-get install libblas3 libblas-dev
# LAPACK
apt-get install liblapack3 liblapack-dev
# PCRE
apt-get install libpcre3 libpcre3-dev
# The following external package is optional but strongly recommended
# GNU Readline
apt-get install libreadline6 libreadline-dev
# ARPACK
apt-get install libarpack2 libarpack2-dev
# cURL
apt-get install curl
# FFTW3
apt-get install libfftw3-3 libfftw3-dev
# FLTK
apt-get install libfltk1.3 libfltk1.3-dev
# fontconfig
apt-get install fontconfig
# FreeType
apt-get install libfreetype6 libfreetype6-dev
# GLPK
apt-get install libglpk40 libglpk-dev
# gl2ps
apt-get install libgl2ps1 libgl2ps-dev
# gnuplot
apt-get install gnuplot
# GraphicsMagick++
apt-get install libgraphicsmagick-q16-3 libgraphicsmagick++1-dev
# HDF5
apt-get install libhdf5-100 libhdf5-dev
# LLVM
apt-get install llvm
# OpenGL
apt-get install freeglut3 freeglut3-dev libgtkglext1 x11proto-gl-dev
# Qhull
apt-get install libqhull7 libqhull-dev
# QRUPDATE
apt-get install libqrupdate1 libqrupdate-dev
# QScintilla
apt-get install libqscintilla2-dev
# Qt
apt-get install libqtcore4 libqt4-network libqtgui4
# SuiteSparse
apt-get install libsuitesparse-dev
# zlib
apt-get install zlib1g zlib1g-dev
# libboost
apt-get install libboost-dev
# libeigen
apt-get install libeigen3-dev
#######################
# 3D Model load related
#######################
# glm
apt-get install libglm-dev 
# glew
apt-get install libglewmx1.13 glew-utils libglew-dbg libglew-dev libglew2.0 libglew1.13
# libsoil
apt-get install libsoil



