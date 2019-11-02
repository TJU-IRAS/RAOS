/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : draw_3d_car.cpp
   Author : tao.jing
   Date   : 19-10-18
   Brief  : 
**************************************************************************/
#include <GL/glut.h>
#include <cmath>
#include <cstdio>
#include "iostream"
#include <png.h>
#include <stdlib.h>
#include <stdio.h>

int screenWidth = 800;
int screenHeight = 800;
int screenDepth = 600;
float PI = 3.14;
float ANGLE = 0.0f;

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

void reshape(GLint w, GLint h);

GLuint loadBMP_custom(const char *imagepath);

void init()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    GLfloat lgt1_diffuse[] = {0.05f, 0.05f, 0.6f, 1.0f};
    GLfloat lgt2_diffuse[] = {0.6f, 0.05f, 0.05f, 1.0f};
    GLfloat light_pos1[] = {5.0f, 5.0f, 0.0f, 1.0f};
    GLfloat light_pos2[] = {-5.0f, 5.0f, 0.0f, 1.0f};

    GLfloat ambientLight[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat diffuseLight[] = {1.0f, 1.f, 1.f, 1.0f};
    GLfloat specularLight[] = {0.1f, 0.1f, 0.1f, 1.0f};
    GLfloat position[] = {50.0f, 1.0f, 1.0f, 1.0f};
    GLfloat shininess[] = {10.0f};
    GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat mat_shininess[] = {50.0};
    glShadeModel(GL_SMOOTH);

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
//    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glLightfv(GL_LIGHT0, GL_DIFFUSE, specularLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);


}

/** https://www.youtube.com/watch?v=NT-0Q2Psp2Y */
void timer(int)
{
    glutPostRedisplay();
    glutTimerFunc(1000 / 60, timer, 0);
    ANGLE += 2.5;
}

void circle(int radius, int width)
{
    int numPoints = 360;
    glColor3ub(253, 165, 15);
    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    glBegin(GL_POLYGON);
    for (int i = 0; i < numPoints; i++)
    {
        float angle = i * (2 * PI / numPoints);
        float x = (float) (cos(angle) * radius);
        float y = (float) (sin(angle) * radius);
        float tx = (float) (cos(angle) * 0.5 + 0.5);
        float ty = (float) (sin(angle) * 0.5 + 0.5);

        glTexCoord2f(tx, ty);
        glNormal3f(0, 0, 1);
        glVertex2f(x, y);
    }
    glEnd();
    glPopMatrix();

}

void circles(int depth, int width, int height)
{
    glPushMatrix();
    glTranslatef(width + 1, 0, 50);
    circle((int) (height * .7), width);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(width + 1, 0, -50);
    circle((int) (height * .7), width);
    glPopMatrix();

}

void wheel(int radius)
{
    glColor3ub(121, 61, 55);
    glutSolidTorus(radius * .45, radius, 360, 360);

    glColor3ub(90, 103, 51);

    for (int i = 0; i <= 5; i++)
    {
        float angle = i * (2 * PI / 5) - ANGLE;
        int x = (int) (cos(angle) * radius);
        int y = (int) (sin(angle) * radius);
        glPushMatrix();
        glRotatef(90, x, y, 0);
        glTranslatef(0, 0, -30);
        glutSolidCone(radius * .2, radius / 1.2, 360, 10);
        glPopMatrix();
    }
}

void wheels(int depth, int width, int height)
{
    depth = depth + 19;
    width = .6 * width;

    glPushMatrix();
    glTranslatef(width, -height, -depth);
    wheel((int) (height * .7));
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-width, -height, -depth);
    wheel((int) (height * .7));
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-width, -height, depth);
    wheel((int) (height * .7));
    glPopMatrix();

    glPushMatrix();
    glTranslatef(width, -height, depth);
    wheel((int) (height * .7));
    glPopMatrix();
}

void up()
{
    int depth = 100;
    int width = 100;
    int height = 30;

    glPushMatrix();
    glTranslated(-100, 3 * height, 0);
    for (int face = 0; face < 6; face++)
    {
        glBegin(GL_POLYGON);
        for (int vertex = 0; vertex < 4; vertex++)
        {
            glNormal3f(1, 0, 0);
            glVertex3i(width * faces[face][vertex][0], height * faces[face][vertex][1], depth * faces[face][vertex][2]);
        }
        glEnd();

    }
    glPopMatrix();
}

void body(int depth, int width, int height)
{
    glColor3ub(0, 61, 55);
    for (int face = 0; face < 6; face++)
    {
        glBegin(GL_POLYGON);
        for (int vertex = 0; vertex < 4; vertex++)
        {
            glNormal3f(1, 0, 0);
            glVertex3i(width * faces[face][vertex][0], height * faces[face][vertex][1], depth * faces[face][vertex][2]);
        }
        glEnd();
    }
}

void car()
{
    int depth = 120;
    int width = 250;
    int height = 60;


    glRotatef(-90, 0, 1, 0);
    body(depth, width, height);
    up();
    wheels(depth, width, height);
    circles(depth, width, height);
    glPopMatrix();


}

void track()
{
    float radian, x, y, xcos, ysin;
    float radius = screenDepth * 0.5f;

    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    glBegin(GL_POLYGON);
    for (int angle = 0; angle < 360; angle += 1)
    {
        radian = angle * (PI / 180.0f);
        xcos = (float) cos(radian);
        ysin = (float) sin(radian);
        x = xcos * radius;
        y = ysin * radius;

        glColor3ub(14, 14, 14);
        glNormal3f(0, 0, 1);
        glVertex2f(x, y);

    }
    glEnd();

    glColor3ub(85, 107, 47);
    GLuint texture = loadBMP_custom("../data/res/bush.bmp");
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_POLYGON);
    for (int angle = 0; angle < 360; angle += 1)
    {
        radian = angle * (PI / 180.0f);
        xcos = (float) cos(radian);
        ysin = (float) sin(radian);
        x = xcos * radius * .5f;
        y = ysin * radius * .5f;

        glNormal3f(0, 0, 1);
        glTexCoord2f(xcos * .5f + .5f, ysin * .5f + .5f);
        glVertex3f(x, y, 1);
    }

    glEnd();
    glPopMatrix();

}

/* https://stackoverflow.com/questions/16525916/2d-texture-or-cube-mapping */
void tree(int height, int base)
{
    glColor3ub(100, 139, 34);

    GLUquadricObj *quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GL_SMOOTH);
    GLuint texture = loadBMP_custom("../data/res/bush.bmp");
    gluQuadricTexture(quadratic, GL_TRUE);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    gluCylinder(quadratic, base, 0, height, 360, 360);
    glPopMatrix();

    glColor3ub(139, 69, 15);

    GLUquadricObj *quadratic2 = gluNewQuadric();
    gluQuadricNormals(quadratic2, GL_SMOOTH);
    texture = loadBMP_custom("../data/res/Barkdrk.bmp");
    gluQuadricTexture(quadratic2, GL_TRUE);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPushMatrix();
    glTranslatef(0, -height, 0);
    glRotatef(-90, 1, 0, 0);

    gluCylinder(quadratic2, .3 * base, .3 * base, height, 360, 360);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);


}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    init();

    gluLookAt(1, 1, 0, 0, 0, 0, 0, 1, 0); // TOP see car up

    glPushMatrix();
    glTranslatef(0, 100, 0);
    tree(200, 100);
    glPopMatrix();


    float r = screenDepth * 2;
    float angle = (float) 2 * PI * ANGLE / 360;
    float x = (float) (cos(angle) * r);
    float y = (float) (sin(angle) * r);

    glPushMatrix();
    glScalef(.2, .2, .2);
    glTranslatef(x, -375, y);
    glRotatef(-ANGLE, 0, 1, 0);
    car();
    glPopMatrix();


    glPushMatrix();
    glTranslatef(0, -100, 0);
    glRotatef(90, 0, 0, 1);
    track();
    glPopMatrix();

    glutSwapBuffers();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitWindowSize(screenWidth, screenHeight);
    glutCreateWindow("car");

    glutInitDisplayMode(GLUT_RGB | GL_DEPTH | GL_DOUBLE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(0, timer, 0);
    glClearColor(0, 0, 0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-screenWidth / 2.0f, screenWidth / 2.0f, -screenHeight / 2.0f, screenHeight / 2.0f, -screenDepth / 2.0f,
            screenDepth / 2.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glutMainLoop();
    return 0;
}

void reshape(GLint w, GLint h)
{
    glViewport(0, 0, w, h);
    GLfloat aspect = GLfloat(w) / GLfloat(h);
    glLoadIdentity();
    if (w <= h)
    {
        glOrtho(-screenWidth / 2.0f, screenWidth / 2.0f, -screenHeight / (2.0f * aspect),
                screenHeight / (2.0f * aspect), -screenDepth / 2.0f, screenDepth / 2.0f);
        glClear(GL_COLOR_BUFFER_BIT);

    } else
    {
        glOrtho(-screenWidth * aspect / 2.0f, screenWidth * aspect / 2.0f, -screenHeight / 2.0f, screenHeight / 2.0f,
                -screenDepth / 2.0f, screenDepth / 2.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    }
}

/** https://stackoverflow.com/questions/12518111/how-to-load-a-bmp-on-glut-to-use-it-as-a-texture */
GLuint loadBMP_custom(const char *imagepath)
{
    // Data read from the header of the BMP file
    unsigned char header[54]; // Each BMP file begins by a 54-bytes header
    unsigned int dataPos;     // Position in the file where the actual data begins
    unsigned int width, height;
    unsigned int imageSize;   // = width*height*3
    // Actual RGB data
    unsigned char *data;
    // Open the file
    FILE *file = fopen(imagepath, "rb");
    if (!file)
    {
        printf("Image could not be opened\n");
        return 0;
    }
    if (fread(header, 1, 54, file) != 54)
    { // If not 54 bytes read : problem
        printf("Not a correct BMP file\n");
        return false;
    }

    if (header[0] != 'B' || header[1] != 'M')
    {
        printf("Not a correct BMP file\n");
        return 0;
    }
    // Read ints from the byte array
    dataPos = *(int *) &(header[0x0A]);
    imageSize = *(int *) &(header[0x22]);
    width = *(int *) &(header[0x12]);
    height = *(int *) &(header[0x16]);

    // Some BMP files are misformatted, guess missing information
    if (imageSize == 0)
        imageSize = width * height * 3; // 3 : one byte for each Red, Green and Blue component
    if (dataPos == 0)
        dataPos = 54; // The BMP header is done that way

    // Create a buffer
    data = new unsigned char[imageSize];

    // Read the actual data from the file into the buffer
    fread(data, 1, imageSize, file);

    //Everything is in memory now, the file can be closed
    fclose(file);

    // Create one OpenGL texture
    GLuint textureID;
    glGenTextures(1, &textureID);

    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);


    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
    free(data);


    return textureID;
}



