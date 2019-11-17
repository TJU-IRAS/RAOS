/*
 * 3D Reference Point Drawing
 *          using OpenGL GLUT
 *
 * This file contains stuff for drawing the reference point for
 * the trajectory control of Robot using OpenGL GLUT.
 * The declarations of the classes or functions are written in
 * file draw_ref_point.h
 *
 * Author: Roice (LUO Bing)
 * Date: 2017-03-21 create this file
 */

#include <GL/glew.h>
#include GL_HEADER
#include <stdio.h>
#include <cmath>
#include <vector>
#include "string.h"
#include "model/robot.h"
#include "ui/draw/materials.h" // use material lists
#include "method/method.h"

/* graphics stuff */
static double RAD2DEG = 180. / M_PI;

static void draw_spinning_mark(int shadow)
{
    float inner_radius = 0.08;
    float outer_radius = 0.1;
    float z1 = inner_radius;
    float z2 = outer_radius;
    float x1 = 0, x2 = 0;
    float newx1, newz1, newx2, newz2;

    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(shadow ? SHADOW_MAT : ROSE_MAT);

    for (float k = 0; k <= 2 * M_PI; k += M_PI / 12)
    {
        newx1 = outer_radius * std::sin(k);
        newz1 = outer_radius * std::cos(k);
        newx2 = inner_radius * std::sin(k);
        newz2 = inner_radius * std::cos(k);
        glBegin(GL_POLYGON);
        glVertex3f(x2, 0.0, z2);
        glVertex3f(x1, 0.0, z1);
        glVertex3f(newx1, 0.0, newz1);
        glVertex3f(newx2, 0.0, newz2);
        glEnd();
        x1 = newx1;
        x2 = newx2;
        z1 = newz1;
        z2 = newz2;
    }
    glBegin(GL_POLYGON);
    glVertex3f(0.01, 0.0, inner_radius);
    glVertex3f(-0.01, 0.0, inner_radius);
    glVertex3f(-0.01, 0.0, -inner_radius);
    glVertex3f(0.01, 0.0, -inner_radius);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(inner_radius, 0., 0.01);
    glVertex3f(-inner_radius, 0., 0.01);
    glVertex3f(-inner_radius, 0., -0.01);
    glVertex3f(inner_radius, 0., -0.01);
    glEnd();

    glPopAttrib();
}

void draw_ref_point(std::vector<Robot *> *robots)
{
    char name[10];
    float gl_pos[3];

    static double phi = 0., theta = 0., psi = 0.;

    psi += 5. * M_PI / 180.;
    if (psi > 2 * M_PI) psi -= 2 * M_PI;
    float mat[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    for (unsigned int idx_robot = 0; idx_robot < robots->size(); idx_robot++)
    {// draw ref point of every robot

        // convert enu frame to GL frame
        gl_pos[0] = robots->at(idx_robot)->ref_state.pos[0];
        gl_pos[1] = robots->at(idx_robot)->ref_state.pos[2];
        gl_pos[2] = -robots->at(idx_robot)->ref_state.pos[1];

#ifdef METHOD_HOVER
        gl_pos[2] = 0.0 ;
#endif
        /* draw a spinning mark to indicate ref point */
        glPushMatrix();
        glTranslatef(gl_pos[0], gl_pos[1], gl_pos[2]);
        /* apply NASA aeroplane Euler angles standard
	     * (in terms of OpenGL X,Y,Z frame where
	     * Euler earth axes X,Y,Z are openGl axes X,Z,-Y)
	     */
        glRotatef(RAD2DEG * psi, 0.0, 1.0, 0.0);
        glRotatef(RAD2DEG * theta, 0.0, 0.0, -1.0);
        glRotatef(RAD2DEG * phi, 1.0, 0.0, 0.0);
        draw_spinning_mark(0);
        glPopMatrix();
        /* draw the on-the-ground shadow of spinning mark */
        glPushMatrix();
        glTranslatef(gl_pos[0], 0.0, gl_pos[2]);
        glMultMatrixf(mat);
        /* apply NASA aeroplane Euler angles standard
	     * (same drill) */
        glRotatef(RAD2DEG * psi, 0.0, 1.0, 0.0);
        glRotatef(RAD2DEG * theta, 0.0, 0.0, -1.0);
        glRotatef(RAD2DEG * phi, 1.0, 0.0, 0.0);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        draw_spinning_mark(1);
        glDisable(GL_BLEND);
        glPopMatrix();


        /* draw index of ref points, count from 1 */
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        snprintf(name, sizeof(name), "%d", idx_robot + 1);
        glColor3f(0.3, 0.3, 0.3); // gray
        gl_font(FL_HELVETICA_BOLD, 12);
        //glRasterPos3fv(gl_pos);
        gl_draw(name, strlen(name));
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
    }

}

/* End of draw_ref_point.cxx */
