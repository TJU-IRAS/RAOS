/*
 * Draw 3d wind vector
 *
 * This file is included by draw_arena.h
 *
 * Author: Zhangqi Kang
 * Date: 2017-11-30 create this file
 */


#include <GL/glew.h>
#include GL_HEADER
#include GLU_HEADER
#include GLUT_HEADER
#include <cmath>
#include <vector>
#include "model/windvector.h"
#include "ui/draw/draw_windvector.h"
#include "ui/draw/materials.h" // use material lists
#include "model/environment.h"
#include "model/plume.h"

using namespace std;

// typedef struct {
//   float pos[3];
//   float vel[3];
//   float r;
// } Vector_state;


std::vector<FilaState_t> *vectorstate = wind_get_vector_state();

#ifndef M_PI
#define M_PI 3.14159265
#endif

float bottom_x = -1.0,
    bottom_y = -1.0,
    top_x = 1.0,
    top_y = 1.0,
    grid_height = 2.0;

void draw_arrow(float start_x, float start_y, float start_z,
                float wind_u, float wind_v, float wind_w, GLuint color);

// void WindVector_Init(float env_width, float env_length, float vector_height, float common_dt)
//  {
//    bottom_x = ceil( - env_width  / 2 );
//    top_x    = ceil(   env_width  / 2 );
//    bottom_y = ceil( - env_length / 2 );
//    top_y    = ceil(   env_length / 2 );
//    grid_height = vector_height;
//
//    //Env_wind_info = new SimEnvInfo(common_dt);
//  }



/*color map
 *yellow  : LAND_MAT
 *black   : STEEL_MAT
 *gray   ： CEMENT_MAT
 *green  :  GRASS_MAT
 *red    :  ROSE_MAT
 */
void WindVector_draw(void)
{
    for (float num = 0; num < vectorstate->size(); num++)
    {
        draw_arrow(vectorstate->at(num).pos[0], vectorstate->at(num).pos[1], vectorstate->at(num).pos[2],
                   vectorstate->at(num).vel[0], vectorstate->at(num).vel[1], vectorstate->at(num).vel[2], ROSE_MAT);
    }
    //  float pos[3],wind[3];
    //  pos[2] = grid_height;
    //  for( float id_x = bottom_x ; id_x < top_x ; id_x++ )
    //  {
    //    for( float id_y = bottom_y ; id_y < top_y ; id_y++ )
    //    {
    //      // 获取风速
    //      pos[0] = id_x ;
    //      pos[1] = id_y;
    //      Env_wind_info->measure_wind(pos, wind);
    //
    //      draw_arrow(id_x,id_y,grid_height,wind[0],wind[1],wind[2],ROSE_MAT);
    //    }
    //  }
}


void draw_arrow(float start_x, float start_y, float start_z,
                float wind_u, float wind_v, float wind_w, GLuint vector_color)
{
    const float WindToArrowRatio = 0.1;
    float arrow_x = wind_u * WindToArrowRatio,
        arrow_y = wind_v * WindToArrowRatio,
        arrow_z = wind_w * WindToArrowRatio;
    float arrow_length = sqrt(arrow_x * arrow_x + arrow_y * arrow_y + arrow_z * arrow_z);
    float end_x = start_x + arrow_x,
        end_y = start_y + arrow_y,
        end_z = start_z + arrow_z;

    float angle_yaw = atan(arrow_y / arrow_x) * 180.0 / M_PI + 90, //偏航
        angle_pitch = atan(arrow_z / sqrt(arrow_x * arrow_x + arrow_y * arrow_y)) * 180.0 / M_PI; //
    if (arrow_x < 0)
        angle_yaw += 180;
    if (arrow_x == 0 && arrow_y == 0)
        angle_yaw = 0;
    //printf("angle_yaw:%f , angle_pitch:%f\n", angle_yaw, angle_pitch);

    glPushMatrix();
    glPushAttrib(GL_LIGHTING_BIT);
    glCallList(vector_color);

    //draw arrow's line
    glBegin(GL_LINES);
    glVertex3f(start_x, start_z, -start_y);
    glVertex3f(end_x, end_z, -end_y);
    glEnd();

    //draw arrow's head
    glTranslatef(end_x, end_z, -end_y); //move cone
    if (arrow_x == 0 && arrow_y == 0)   //spin cone
        glRotated(-angle_pitch, 1, 0, 0);
    else
    {
        glRotated(angle_yaw, 0, 1, 0); //rotate by axis Z
        glRotated(-angle_pitch, arrow_x, 0, arrow_y);
        //  printf("angle_pitch:%f\n",angle_pitch);
    }
    glutSolidCone(0.3 * arrow_length, 0.6 * arrow_length, 16, 16);//cone : 1:radius ; 2:height ; grid ; grid.

    glPopMatrix();
    glPopAttrib();
}
