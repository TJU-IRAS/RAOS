/*
 * 3D Robots Drawing
 *          using OpenGL GLUT
 *
 * This file contains stuff for drawing the robots for 3D
 * Robot Active Olfaction using OpenGL GLUT.
 * The declarations of the classes or functions are written in
 * file draw_robots.h, which is included by DrawScene.h
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-03-09 create this file
 */

#include <GL/glew.h>
#include GL_HEADER
#include <vector>
#include "model/robot.h"
#include "model/quadrotor.h"
#include "ui/draw/draw_qr.h"
#include "ui/draw/draw_ground_robot.h"

void draw_robots(std::vector<Robot *> *robots)
{
    for (unsigned int idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {// draw every robot

        /* draw robot according to its type, configures... */
        if (robots->at(idx_robot)->config.type == quadrotor)
        {
            // draw quadrotor

            draw_qr(&(robots->at(idx_robot)->state),
                    (QRframe_t *) (robots->at(idx_robot)->config.frame));

/*
            GRframe_t frame;
            draw_gr(&(robots->at(idx_robot)->state),
                    &frame);*/

        }

        /* draw robot according to its type, configures... */
        if (robots->at(idx_robot)->config.type == ground_robot)
        {
            // draw ground robot
            draw_gr(&(robots->at(idx_robot)->state),
                    (GRframe_t *) (robots->at(idx_robot)->config.frame));
        }
    }
}

/* End of draw_robots.cxx */
