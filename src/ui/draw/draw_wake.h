/*
 * Quadrotor's FVM result drawing
 *          using OpenGL GLUT
 *
 * This file contains stuff to draw the wake of a quadrotor
 * using OpenGL GLUT.
 * The implementations of the classes or functions are written in 
 * file draw_wake.cxx, which is included by DrawScene.h
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-07 create this file
 */
#ifndef DRAW_WAKE_H
#define DRAW_WAKE_H

#include <vector>
#include "model/robot.h"

void draw_wake(std::vector<Robot *> *);

#endif
