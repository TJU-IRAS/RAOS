/*
 * 3D RAO Sim Scene Drawing
 *          using OpenGL GLUT
 *
 * This file implements stuff for drawing the simulation scene for 3D
 * Robot Active Olfaction using OpenGL GLUT.
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-23 create this file
 */

#include <GL/glew.h>
#include GL_HEADER

/* thread */
#include <pthread.h>

#include "SimThread.h"
#include "model/SimModel.h"
#include "ui/draw/draw_robots.h" // robots visualization
#include "ui/draw/draw_arena.h" // arena visualization
#include "ui/draw/draw_plume.h" // plume visual

#ifdef RAOS_FEATURE_WAKES

#include "ui/draw/draw_wake.h" // wake visual

#endif

#include "ui/draw/draw_ref_point.h"
#include "ui/draw/materials.h" // create material lists
#include "ui/draw/cad/loadmodel.h"
#include "ui/draw/draw_windvector.h"
#include "SimConfig.h"
#include "method/method.h"


GLfloat localAmb[4] = {0.7, 0.7, 0.7, 1.0};
GLfloat ambient0[4] = {0.0, 0.0, 0.0, 1.0};
GLfloat diffuse0[4] = {1.0, 1.0, 1.0, 1.0};
GLfloat specular0[4] = {1.0, 0.0, 0.0, 1.0};
GLfloat ambient1[4] = {0.0, 0.0, 0.0, 1.0};
GLfloat diffuse1[4] = {1.0, 1.0, 1.0, 1.0};
GLfloat specular1[4] = {1.0, 0.0, 0.0, 1.0};
GLfloat position0[4] = {2.0, 100.5, 1.5, 1.0};
GLfloat position1[4] = {-2.0, 100.5, 1.0, 0.0};

/*--------------------------------------------------------------------------
 * draw everything (environment + quad rotor)
 *--------------------------------------------------------------------------
 */
void DrawScene(void)
{
    /* GL stuff */
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, localAmb);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_POSITION, position0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient1);
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glEnable(GL_LIGHTING); // default enable for the rest routines

    /* draw arena */
    draw_arena(SIM_ARENA_OBJ);
    //draw_arena(SIM_ARENA_BASIC);

    /* draw wind vector */
#ifdef DRAW_VECTOR_IN_SCENE
    WindVector_draw();
#endif

    if (!sim_is_running_or_not())
        return;

    pthread_mutex_lock(sim_get_data_lock());

    /* draw quadrotor */
#ifndef METHOD_HOVER
    draw_ref_point(SimModel_get_robots());
#endif
    draw_robots(SimModel_get_robots());

#ifdef RAOS_FEATURE_WAKES
    /* draw rotor wakes */
    draw_wake(SimModel_get_robots());
#endif

    /* draw plume */
    draw_plume();

    pthread_mutex_unlock(sim_get_data_lock());
}

void DrawScene_init(void) // call before DrawScene
{
    create_materials();
    loadmodel_init();
    //SimConfig_t *config = SimConfig_get_configs();
    //WindVector_Init(config->arena.w, config->arena.l, 1.5 , config->common.dt);
}
/* End of DrawScene.cxx */
