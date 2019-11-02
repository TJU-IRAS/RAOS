/*
 * 3D Plume drawing
 *          using OpenGL GLUT
 *
 * This file contains stuff for drawing plume for 3D
 * Robot Active Olfaction using OpenGL GLUT.
 * The declarations of the classes or functions are written in
 * file draw_plume.h, which is included by DrawScene.h
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-26 create this file
 */
#include <GL/glew.h>
#include GL_HEADER
#include GLUT_HEADER
#include <vector>
#include "model/plume.h"
#include "ui/draw/materials.h"

void draw_plume(void)
{
    std::vector<FilaState_t> *fs = plume_get_fila_state();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (unsigned int i = 0; i < fs->size(); i++) // for each filament
    {
        glPushMatrix();
        glTranslatef(fs->at(i).pos[0], fs->at(i).pos[2], -fs->at(i).pos[1]);
        glPushAttrib(GL_LIGHTING_BIT);
        SimMaterial_chlorine(0.5 + 0.5 * (1.0 - 10 * fs->at(i).r));
        //SimMaterial_smoke(1.0-10*fs->at(i).r);
        glutSolidSphere(fs->at(i).r, 8, 3);
        glPopAttrib();
        glPopMatrix();
    }
    glDisable(GL_BLEND);
}
/* End of draw_plume.cxx */
