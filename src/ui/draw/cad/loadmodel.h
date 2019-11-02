#ifndef _LOADMODEL_H_
#define _LOADMODEL_H_

// 引入GLEW库 定义静态链接
#define GLEW_STATIC

#include <GL/glew.h>
// 引入GLFW库
#include <GLFW/glfw3.h>
// 引入SOIL库
#include <SOIL/SOIL.h>
// 引入GLM库
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <cstdlib>

// 包含着色器加载库
#include "shader.h"
// 包含相机控制辅助类
#include "camera.h"
// 包含纹理加载类
#include "texture.h"
// 加载模型的类
#include "model.h"

// 键盘回调函数原型声明
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

// 鼠标移动回调函数原型声明
void mouse_move(double xpos, double ypos);

// 鼠标滚轮回调函数原型声明
void mouse_scroll(double xoffset, double yoffset);// 场景中移动
void do_movement(unsigned char key);

bool loadmodel_init(void);

bool loadmodel_update(void);

#endif
