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
#include <iostream>

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

GLuint texWall;

#define BMP_Header_Length 54  //图像数据在内存块中的偏移量
static GLfloat angle = 0.0f;   //旋转角度

GLuint load_texture(const char* file_name);

/*--------------------------------------------------------------------------
 * draw everything (environment + quad rotor)
 *--------------------------------------------------------------------------
 */

void draw_buildings();

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

    draw_buildings();

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

void paint_building_roof(float* x, float* y, float height)
{
    //std::cout << "X: " << x[0] << " " << x[1] << std::endl;
    //std::cout << "Y: " << y[0] << " " << y[1] << std::endl;

    glCallList(GR_WHEEL);
    glBegin(GL_POLYGON);
    glVertex3f(x[0], height, y[0]);
    glVertex3f(x[0], height, y[1]);
    glVertex3f(x[1], height, y[1]);
    glVertex3f(x[1], height, y[0]);
    glEnd();

    /*
    glBindTexture(GL_TEXTURE_2D, texWall);
    glBegin(GL_POLYGON);
    glTexCoord2f(0.0f, 0.0f);glVertex3f(x[0], height, y[0]);
    glTexCoord2f(0.0f, 3.0f);glVertex3f(x[0], height, y[1]);
    glTexCoord2f(3.0f, 3.0f);glVertex3f(x[1], height, y[1]);
    glTexCoord2f(3.0f, 0.0f);glVertex3f(x[1], height, y[0]);
    glEnd();
    */
}

void draw_buildings()
{
    float x_offset = 15.0f;
    float y_offset = 15.0f;
    float z_offset = 0.0f;

    float x_range[3][2] = {
        {9.0f, 11.0f}, {14.0f, 16.0f}, {19.0f, 21.0f}
    };
    float y_range[3][2] = {
        {9.0f, 11.0f}, {14.0f, 16.0f}, {19.0f, 21.0f}
    };
    float z_range[1][2] = {{0.0f, 4.0f}};

    for(unsigned int idx = 0; idx < sizeof(x_range) / sizeof(float); idx++)
    {
        reinterpret_cast<float*>(x_range)[idx] -= x_offset;
    }
    for(unsigned int idx = 0; idx < sizeof(y_range) / sizeof(float); idx++)
    {
        reinterpret_cast<float*>(y_range)[idx] -= y_offset;
    }
    for(unsigned int idx = 0; idx < sizeof(z_range) / sizeof(float); idx++)
    {
        reinterpret_cast<float*>(z_range)[idx] -= z_offset;
    }

    float building_height = 4.0f;

    for (unsigned int row_idx = 0; row_idx < 3; row_idx++)
    {
        for (unsigned int col_idx = 0; col_idx < 3; col_idx++)
        {
            paint_building_roof(reinterpret_cast<float*>(x_range[row_idx]), reinterpret_cast<float*>(y_range[col_idx]), building_height);
        }
    }

    /*
    static GLint imagewidth;
    static GLint imageheight;
    static GLint pixellength;
    static GLubyte* pixeldata;

    FILE* pfile=fopen("beauty.bmp","rb");
    if(pfile == 0) exit(0);
    //读取图像大小
    fseek(pfile,0x0012,SEEK_SET);
    fread(&imagewidth,sizeof(imagewidth),1,pfile);
    fread(&imageheight,sizeof(imageheight),1,pfile);
    //计算像素数据长度
    pixellength=imagewidth*3;
    while(pixellength%4 != 0)pixellength++;
    pixellength *= imageheight;
    //读取像素数据
    pixeldata = (GLubyte*)malloc(pixellength);
    if(pixeldata == 0) exit(0);
    fseek(pfile,54,SEEK_SET);
    fread(pixeldata,pixellength,1,pfile);

    //关闭文件
    fclose(pfile);
    glDrawPixels(imagewidth,imageheight,GL_BGR_EXT,GL_UNSIGNED_BYTE,pixeldata);
    glFlush();
    free(pixeldata);
    */

    //glEnable(GL_TEXTURE_2D);
    //texWall = load_texture("beauty.bmp");

}


// 函数power_of_two用于判断一个整数是不是2的整数次幂
int power_of_two(int n)
{
    if( n <= 0 )
        return 0;
    return (n & (n-1)) == 0;
}

GLuint load_texture(const char* file_name)
{
    GLint width, height, total_bytes;
    GLubyte* pixels = 0;
    GLuint last_texture_ID=0, texture_ID = 0;

    // 打开文件，如果失败，返回
    FILE* pFile = fopen(file_name, "rb");
    if( pFile == 0 )
        return 0;

    // 读取文件中图象的宽度和高度
    fseek(pFile, 0x0012, SEEK_SET);
    fread(&width, 4, 1, pFile);
    fread(&height, 4, 1, pFile);
    fseek(pFile, BMP_Header_Length, SEEK_SET);

    // 计算每行像素所占字节数，并根据此数据计算总像素字节数
    {
        GLint line_bytes = width * 3;
        while( line_bytes % 4 != 0 )
            ++line_bytes;
        total_bytes = line_bytes * height;
    }

    // 根据总像素字节数分配内存
    pixels = (GLubyte*)malloc(total_bytes);
    if( pixels == 0 )
    {
        fclose(pFile);
        return 0;
    }

    // 读取像素数据
    if( fread(pixels, total_bytes, 1, pFile) <= 0 )
    {
        free(pixels);
        fclose(pFile);
        return 0;
    }

    // 对就旧版本的兼容，如果图象的宽度和高度不是的整数次方，则需要进行缩放
    // 若图像宽高超过了OpenGL规定的最大值，也缩放
    {
        GLint max;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max);
        if( !power_of_two(width)
            || !power_of_two(height)
            || width > max
            || height > max )
        {
            const GLint new_width = 256;
            const GLint new_height = 256; // 规定缩放后新的大小为边长的正方形
            GLint new_line_bytes, new_total_bytes;
            GLubyte* new_pixels = 0;

            // 计算每行需要的字节数和总字节数
            new_line_bytes = new_width * 3;
            while( new_line_bytes % 4 != 0 )
                ++new_line_bytes;
            new_total_bytes = new_line_bytes * new_height;

            // 分配内存
            new_pixels = (GLubyte*)malloc(new_total_bytes);
            if( new_pixels == 0 )
            {
                free(pixels);
                fclose(pFile);
                return 0;
            }

            // 进行像素缩放
            gluScaleImage(GL_RGB,
                          width, height, GL_UNSIGNED_BYTE, pixels,
                          new_width, new_height, GL_UNSIGNED_BYTE, new_pixels);

            // 释放原来的像素数据，把pixels指向新的像素数据，并重新设置width和height
            free(pixels);
            pixels = new_pixels;
            width = new_width;
            height = new_height;
        }
    }

    // 分配一个新的纹理编号
    glGenTextures(1, &texture_ID);
    if( texture_ID == 0 )
    {
        free(pixels);
        fclose(pFile);
        return 0;
    }

    // 绑定新的纹理，载入纹理并设置纹理参数
    // 在绑定前，先获得原来绑定的纹理编号，以便在最后进行恢复
    GLint lastTextureID=last_texture_ID;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &lastTextureID);
    glBindTexture(GL_TEXTURE_2D, texture_ID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                 GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, lastTextureID);  //恢复之前的纹理绑定
    free(pixels);
    return texture_ID;
}

/* End of DrawScene.cxx */
