/*
 * Materials for drawing using OpenGL
 *
 * This file defines DisplayLists enum. The lists creation are written in
 * material.cxx
 * This file is included by draw_qr.cxx draw_arena.cxx ... to set materials
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-24 create this file
 */

typedef enum
{
    LAND_MAT,    //yellow
    STEEL_MAT,   //black
    CEMENT_MAT,  //gray
    GRASS_MAT,   //green:
    SMOKE_MAT,   //black
    SHADOW_MAT,  //black
    VORTICE_MAT, //gray
    CHLORINE_MAT,//green
    ROSE_MAT,    //red
    PURPLE_MAT,  //purple
    GR_BODY,     //ground robot body
    GR_UP,       //ground robot up
    GR_WHEEL     //ground wheel
} DisplayLists;

void create_materials(void);

void SimMaterial_smoke(float);

void SimMaterial_chlorine(float);
/* End of materials.h */
