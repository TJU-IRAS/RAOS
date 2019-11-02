/*
 * Arena Drawing
 *
 * This file contains stuff for drawing an arena for 3D
 * Robot Active Olfaction using OpenGL GLUT.
 * The declarations of the classes or functions are written in
 * file draw_arena.h, which is included by DrawScene.h
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-24 create this file
 */

#include <GL/glew.h>
#include GL_HEADER
#include GLU_HEADER
#include GLUT_HEADER
#include <cmath> // floor()
#include "ui/draw/draw_arena.h"
#include "ui/draw/materials.h" // use material lists
#include "SimConfig.h" // get configurations about Arena
#include "ui/draw/cad/loadmodel.h"

/* declarations of local functions */
// functions to create arenas
static void draw_arena_basic(void);

static void draw_arena_obj(void);

void draw_arena(int arena_name)
{
    switch (arena_name)
    {
        case SIM_ARENA_BASIC:
            draw_arena_basic();
            break;
        case SIM_ARENA_OBJ:
            draw_arena_obj();
            //draw_arena_basic();
            break;
        default:
            draw_arena_basic();
    }
}


/* functions to draw arenas , and display 3D environment model in .obj format*/
static void draw_arena_obj(void)
{
    /* get configs of arena */
    SimConfig_t *config = SimConfig_get_configs();
    // calculate the four vertex of arena
    float grid_height = 0.05;
    GLfloat va[3] = {config->arena.w / (float) 2.0, grid_height, -config->arena.l / (float) 2.0},
        //vb[3] = {-config->arena.w / (float) 2.0, grid_height, -config->arena.l / (float) 2.0},
        vc[3] = {-config->arena.w / (float) 2.0, grid_height, config->arena.l / (float) 2.0},
        vd[3] = {config->arena.w / (float) 2.0, grid_height, config->arena.l / (float) 2.0};

    /* draw 3D environment model */
    glCallList(CEMENT_MAT);
    //glBegin(GL_POLYGON);
    loadmodel_update();
    glEnd();

    /* draw grid */
    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(GRASS_MAT);
    glPushMatrix();
    glTranslatef(0, -0.01, 0); // not 0 to avoid conflict with other objs
    //glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_LINES);
    float e;
    for (e = -floor(va[0]);
         e <= floor(va[0]); e += 1.0)
    {
        glVertex3f(e, grid_height, va[2]);
        glVertex3f(e, grid_height, vd[2]);
    }
    for (e = -floor(vd[2]);
         e <= floor(vd[2]); e += 1.0)
    {
        glVertex3f(vc[0], grid_height, e);
        glVertex3f(vd[0], grid_height, e);
    }
    glEnd();
    glDisable(GL_BLEND);
    glPopMatrix();
    glPopAttrib();

    /* draw chimney (odor source), a cylinder */
    GLUquadricObj *chimney_obj = gluNewQuadric();
    gluQuadricDrawStyle(chimney_obj, GLU_FILL);
    gluQuadricNormals(chimney_obj, GLU_SMOOTH);
    glPushMatrix();
    glTranslatef(config->source.x, 0, -config->source.y);
    glRotatef(-90, 1, 0, 0); // make it upright
    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(CEMENT_MAT);
    gluCylinder(chimney_obj,
                0.2, // base radius
                0.1, // top radius
                config->source.z, // length
                8, /*slices*/
                3 /*stacks*/);
    glPopAttrib();
    glPopMatrix();
}


/* functions to draw arenas */
static void draw_arena_basic(void)
{
    /* get configs of arena */
    SimConfig_t *config = SimConfig_get_configs();

    /* draw Ground */
    // calculate the four vertex of ground
    GLfloat va[3] = {config->arena.w / (float) 2.0, 0, -config->arena.l / (float) 2.0},
        vb[3] = {-config->arena.w / (float) 2.0, 0, -config->arena.l / (float) 2.0},
        vc[3] = {-config->arena.w / (float) 2.0, 0, config->arena.l / (float) 2.0},
        vd[3] = {config->arena.w / (float) 2.0, 0, config->arena.l / (float) 2.0};
    glPushMatrix();
    glTranslatef(0, -0.02, 0); // not 0 to avoid conflict with other objs
    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(LAND_MAT);
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3fv(va);
    glVertex3fv(vb);
    glVertex3fv(vc);
    glVertex3fv(vd);
    glEnd();

    glPopAttrib();
    glPopMatrix();

    /* draw grid */
    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(GRASS_MAT);
    glPushMatrix();
    glTranslatef(0, -0.01, 0); // not 0 to avoid conflict with other objs
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_LINES);
    float e;
    for (e = -floor(va[0]);
         e <= floor(va[0]); e += 1.0)
    {
        glVertex3f(e, 0, va[2]);
        glVertex3f(e, 0, vd[2]);
    }
    for (e = -floor(vd[2]);
         e <= floor(vd[2]); e += 1.0)
    {
        glVertex3f(vc[0], 0, e);
        glVertex3f(vd[0], 0, e);
    }
    glEnd();
    glDisable(GL_BLEND);
    glPopMatrix();
    glPopAttrib();

    /* draw chimney (odor source), a cylinder */
    GLUquadricObj *chimney_obj = gluNewQuadric();
    gluQuadricDrawStyle(chimney_obj, GLU_FILL);
    gluQuadricNormals(chimney_obj, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(config->source.x, 0, -config->source.y);
    glRotatef(-90, 1, 0, 0); // make it upright
    glPushAttrib(GL_LIGHTING_BIT);

    glCallList(CEMENT_MAT);
    gluCylinder(chimney_obj,
                0.2, // base radius
                0.1, // top radius
                config->source.z, // length
                8, /*slices*/ 3 /*stacks*/);
    glPopAttrib();
    glPopMatrix();
}

/* End of draw_arena.cxx */
