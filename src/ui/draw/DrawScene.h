/*
 * 3D RAO Sim Scene Drawing
 *          using OpenGL GLUT
 *
 * This file defines stuff for drawing the simulation scene for 3D
 * Robot Active Olfaction using OpenGL GLUT.
 * This file is included by SimView.cxx, and the implementations are
 * written in DrawScene.cxx, which contains functions of files 
 * draw_qr.cxx, draw_pl.cxx, draw_wind.cxx
 *
 * draw_qr.cxx  OpenGL drawing of a quadrotor
 * draw_pl.cxx  OpenGL drawing of plume filaments (Farrell's model)
 * draw_wind.cxx OpenGL drawing of wind vectors
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-23 create this file
 */

#ifndef DRAWSCENE_H
#define DRAWSCENE_H

void DrawScene();

void DrawScene_init();

#endif
/* End of DrawScene.h */
