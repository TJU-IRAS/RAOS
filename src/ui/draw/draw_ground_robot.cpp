/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : draw_ground_robot.cpp
   Author : tao.jing
   Date   : 19-10-18
   Brief  : 
**************************************************************************/
#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>
#include GL_HEADER
#include "draw_ground_robot.h"
#include "ui/draw/materials.h"


/* graphics stuff */
static double RAD2DEG = 180 / M_PI;

int faces[6][4][3] = {
    {
        {1,  -1, -1},
        {-1, -1, -1},
        {-1, 1,  -1},
        {1,  1,  -1}
    },
    {
        {1,  -1, 1},
        {-1, -1, 1},
        {-1, 1,  1},
        {1,  1,  1}
    },
    {
        {-1, -1, 1},
        {-1, -1, -1},
        {-1, 1,  -1},
        {-1, 1,  1}
    },
    {
        {1,  -1, 1},
        {1,  -1, -1},
        {1,  1,  -1},
        {1,  1,  1}
    },
    {
        {1,  -1, 1},
        {1,  -1, -1},
        {-1, -1, -1},
        {-1, -1, 1}
    },
    {
        {1,  1,  1},
        {1,  1,  -1},
        {-1, 1,  -1},
        {-1, 1,  1}
    }
};

void car(RobotState_t *state, GRframe_t *frame);

void draw_ground_robot_model(RobotState_t *state, GRframe_t *frame);

void draw_gr(RobotState_t *state, GRframe_t *frame)
{
    double phi, theta, psi;
    double glX, glY, glZ;
    float mat[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    /* change from NASA airplane to OpenGL coordinates */
    glX = state->pos[0];
    glZ = -state->pos[1];
    //glY = state->pos[2];
    glY = ( frame->body_height + 0.25 ) / 2;

    /* draw the ground robot */
    glPushMatrix();

    /* translate to pos of ground robot */
    glTranslatef(glX, glY, glZ);

    draw_ground_robot_model(state, frame);

    glPopMatrix();
}

void draw_ground_robot_model(RobotState_t *state, GRframe_t *frame)
{
    float r = 600 * 2;
    float angle = (float) 2 * M_PI * frame->angle / 360;
    float x = (float) (cos(angle) * r);
    float y = (float) (sin(angle) * r);


    frame->angle = state->att[2];
    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    glRotatef(frame->angle, 0, 1, 0);
    car(state, frame);
    glPopMatrix();

}

void Cube()
{
    glBegin(GL_QUAD_STRIP);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 1.0f, -1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();
    glBegin(GL_QUAD_STRIP);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glVertex3f(1.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glEnd();
}

void Circle()
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, 0.0f);
    int i = 0;
    for (i = 0; i <= 390; i += 15)
    {
        float p = i * 3.14 / 180;
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
}

void Cylinder(float radius, float height)
{
    glBegin(GL_QUAD_STRIP);
    int i = 0;
    for (i = 0; i <= 390; i += 15)
    {
        float p = i * 3.14 / 180;
        glVertex3f(radius * sin(p), radius * cos(p), height);
        glVertex3f(radius * sin(p), radius * cos(p), 0.0f);
    }
    glEnd();
    Circle();
    glTranslatef(0, 0, height);
    Circle();
}

void wheels(RobotState_t *state, GRframe_t *frame)
{
    float depth = frame->body_depth ;
    float width = frame->body_width ;
    float height = frame->body_height;

    width = width * 1.1f;

    glPushMatrix();
    glRotatef(90.0f, 0.0, 1.0, 0.0);

    glPushMatrix();
    glTranslatef(width, -height, -depth + 0.25f);
    Cylinder(height * 0.7f, 0.5f);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-width, -height, -depth + 0.25f);
    Cylinder(height * 0.7f, 0.5f);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-width, -height, depth - 0.75);
    Cylinder(height * 0.7f, 0.5f);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(width, -height, depth - 0.75);
    Cylinder(height * 0.7f, 0.5f);
    glPopMatrix();

    glPopMatrix();
}

void body(RobotState_t *state, GRframe_t *frame)
{
    float depth = frame->body_depth;
    float width = frame->body_width;
    float height = frame->body_height;

    glCallList(GR_BODY);
    for (int face = 0; face < 6; face++)
    {
        glBegin(GL_POLYGON);
        for (int vertex = 0; vertex < 4; vertex++)
        {
            glNormal3f(1, 0, 0);
            glVertex3f(width * faces[face][vertex][0], height * faces[face][vertex][1], depth * faces[face][vertex][2]);
        }
        glEnd();
    }
}

void up(RobotState_t *state, GRframe_t *frame)
{
    float depth = frame->up_depth ;
    float width = frame->up_width ;
    float height = frame->up_height;

    glPushMatrix();
    glCallList(GR_UP);
    glTranslated(0, frame->up_offset, 0);
    for (int face = 0; face < 6; face++)
    {
        glBegin(GL_POLYGON);
        for (int vertex = 0; vertex < 4; vertex++)
        {
            glNormal3f(1, 0, 0);
            glVertex3f(width * faces[face][vertex][0], height * faces[face][vertex][1], depth * faces[face][vertex][2]);
        }
        glEnd();

    }
    glPopMatrix();
}

void car(RobotState_t *state, GRframe_t *frame)
{
    float depth = 1.5;
    float width = 2.5;
    float height = 1;

    glRotatef(-90, 0, 1, 0);
    body(state, frame);
    up(state, frame);
    wheels(state, frame);
}
