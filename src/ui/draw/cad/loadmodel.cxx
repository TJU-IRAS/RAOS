#include "ui/draw/cad/loadmodel.h"

// 定义程序常量
const int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 600;
// 用于相机交互参数
GLfloat lastX = WINDOW_WIDTH / 2.0f, lastY = WINDOW_HEIGHT / 2.0f;
bool firstMouseMove = true;
bool keyPressedStatus[1024]; // 按键情况记录
GLfloat deltaTime = 0.0f; // 当前帧和上一帧的时间差
GLfloat lastFrame = 0.0f; // 上一帧时间
Camera camera(glm::vec3(0.0f, 1.0f, 3.0f));
glm::vec3 lampPos(0.5f, 10.0f, 10.0f);//光源位置

//global declaration
Model objModel;

bool loadmodel_init(void)
{
    if (!glfwInit())    // 初始化glfw库
    {
        std::cout << "Error::GLFW could not initialize GLFW!" << std::endl;
        return false;
    }

    // 开启OpenGL 3.3 core profile
    //std::cout << "Start OpenGL core profile version 3.3" << std::endl;
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    //GLFWwindow* window = glfwCreateWindow(800, 600, "Robot Active Olfaction Simulator2", NULL, NULL);
    //glfwMakeContextCurrent(window);
    //glfwSetKeyCallback(window, key_callback);
    //glfwSetCursorPosCallback(window, mouse_move_callback);
    //glfwSetScrollCallback(window, mouse_scroll_callback);
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // 初始化GLEW 获取OpenGL函数/home/kang/workspace/RAOS-RAOS_v1.2/src/ui/draw/cad/singapo.obj
    glewExperimental = GL_TRUE; // 让glew获取所有拓展函数
    GLenum status = glewInit();
    if (status != GLEW_OK)
    {
        std::cout << "Error::GLEW glew version:" << glewGetString(GLEW_VERSION)
                  << " error string:" << glewGetErrorString(status) << std::endl;
        glfwTerminate();
        std::system("pause");
        return false;
    }

    // 设置视口参数
    //glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    //Section1 加载模型数据 为了方便更换declaration模型 我们从文件读取模型文件路径
    std::ifstream modelPath("../data/model/model_index.txt");
    if (!modelPath)
    {
        std::cerr << "Error::could not read model path file." << std::endl;
        glfwTerminate();
        std::system("pause");
        return false;
    }
    std::string modelFilePath;
    std::getline(modelPath, modelFilePath);
    if (modelFilePath.empty())
    {
        std::cerr << "Error::model path empty." << std::endl;
        glfwTerminate();
        std::system("pause");
        return false;
    }
    if (!objModel.loadModel(modelFilePath))
    {
        glfwTerminate();
        std::system("pause");
        return false;
    }
    return true;
}


bool loadmodel_update(void)
{
    static Shader shader("../src/ui/draw/cad/model.vertex", "../src/ui/draw/cad/model.frag");

    // Section2 准备着色器程序
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    GLfloat currentFrame = (GLfloat) glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    glfwPollEvents(); // 处理例如鼠标 键盘等事件
    //do_movement(); // 根据用户操作情况 更新相机属性

    // 清除颜色缓冲区 重置为指定颜色
    //glClearColor(0.18f, 0.04f, 0.14f, 1.0f);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader.use();

    // Add a dot lights
    GLint lightAmbientLoc = glGetUniformLocation(shader.programId, "light.ambient");
    GLint lightDiffuseLoc = glGetUniformLocation(shader.programId, "light.diffuse");
    GLint lightSpecularLoc = glGetUniformLocation(shader.programId, "light.specular");
    GLint lightPosLoc = glGetUniformLocation(shader.programId, "light.position");
    GLint attConstant = glGetUniformLocation(shader.programId, "light.constant");
    GLint attLinear = glGetUniformLocation(shader.programId, "light.linear");
    GLint attQuadratic = glGetUniformLocation(shader.programId, "light.quadratic");
    glUniform3f(lightAmbientLoc, 0.2f, 0.2f, 0.2f);
    glUniform3f(lightDiffuseLoc, 0.5f, 0.5f, 0.5f);
    glUniform3f(lightSpecularLoc, 1.0f, 1.0f, 1.0f);
    glUniform3f(lightPosLoc, lampPos.x, lampPos.y, lampPos.z);
    // 设置衰减系数
    glUniform1f(attConstant, 1.0f);
    glUniform1f(attLinear, 0.09f);
    glUniform1f(attQuadratic, 0.032f);
    // 设置观察者位置
    GLint viewPosLoc = glGetUniformLocation(shader.programId, "viewPos");
    glUniform3f(viewPosLoc, camera.position.x, camera.position.y, camera.position.z);

    glm::mat4 projection = glm::perspective(camera.mouse_zoom,
                                            (GLfloat) (WINDOW_WIDTH) / WINDOW_HEIGHT, 1.0f, 100.0f); // 投影矩阵
    glm::mat4 view = camera.getViewMatrix(); // 视变换矩阵
    glUniformMatrix4fv(glGetUniformLocation(shader.programId, "projection"),
                       1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shader.programId, "view"),
                       1, GL_FALSE, glm::value_ptr(view));
    glm::mat4 model;
    model = glm::translate(model, glm::vec3(0.0f, 0.0f, 2.0f)); // 适当调整位置
    model = glm::scale(model, glm::vec3(0.2f, 0.2f, 0.2f)); // 适当缩小模型
    glUniformMatrix4fv(glGetUniformLocation(shader.programId, "model"),
                       1, GL_FALSE, glm::value_ptr(model));
    // 这里填写场景绘制代码
    objModel.draw(shader); // 绘制物体

    glBindVertexArray(0);
    glUseProgram(0);
    //glfwSwapBuffers(window); // 交换缓存

    return true;
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key >= 0 && key < 1024)
    {
        if (action == GLFW_PRESS)
            keyPressedStatus[key] = true;
        else if (action == GLFW_RELEASE)
            keyPressedStatus[key] = false;
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GL_TRUE); // 关闭窗口
    }
}

void mouse_move(double xpos, double ypos)
{
    if (firstMouseMove) // 首次鼠标移动
    {
        lastX = xpos;
        lastY = ypos;
        firstMouseMove = false;
    }

    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    camera.handleMouseMove(xoffset, yoffset);
}

// 由相机辅助类处理鼠标滚轮控制
void mouse_scroll(double xoffset, double yoffset)
{
    camera.handleMouseScroll(yoffset);
}


// 由相机辅助类处理键盘控制
void do_movement(unsigned char key)
{
    //printf("%c",key );
    switch (key)
    {
        case 'w':
        case 'W':
            camera.handleKeyPress(FORWARD, deltaTime);
            break;
        case 's':
        case 'S':
            camera.handleKeyPress(BACKWARD, deltaTime);
            break;
        case 'a':
        case 'A':
            camera.handleKeyPress(LEFT, deltaTime);
            break;
        case 'd':
        case 'D':
            camera.handleKeyPress(RIGHT, deltaTime);
            break;
    }
}
