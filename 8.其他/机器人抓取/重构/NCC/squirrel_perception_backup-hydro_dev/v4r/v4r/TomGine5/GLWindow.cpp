/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2014, Simon Schreiberhuber
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author simon.schreiberhuber
 *
 */
#include "GLWindow.h"
#include <GLFW/glfw3.h>

#include <stdio.h>
using namespace tg;

bool GLWindow::glfwRunning=false;
std::list<GLWindow*> GLWindow::windows;


void GLWindow::errorCallback(int error, const char *description)
{
    fputs(description, stderr);
}

GLWindow::GLWindow(GLuint width, GLuint height, std::string title, GLWindow *share)
{
    if(!glfwRunning){
        if (!glfwInit())
        exit(EXIT_FAILURE);
        glfwRunning = true;
        setErrorCallback(&GLWindow::errorCallback);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);  // yes, 3 and 2!!!
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* shareWindow=0;
    if(share !=0)
        shareWindow=share->window;

    window = glfwCreateWindow(width, height, title.c_str(), NULL, shareWindow);


    if(!window){
        printf("couldn't make window\n");
        exit(EXIT_FAILURE);
    }

    GLWindow::windows.push_back(this);
    keyCallbackPointer = 0;
    cursorCallbackPointer = 0;
    mouseButtonCallbackPointer = 0;
    glfwSetKeyCallback(window,&GLWindow::keyCallback );
    glfwSetCursorPosCallback(window, &GLWindow::cursorCallback);
    glfwSetMouseButtonCallback(window,&GLWindow::mouseButtonCallback);


    //this part seems to be necessary for the linux implementation.
    makeContextCurrent();
    glViewport(0, 0, width, height);

}

GLWindow::~GLWindow()
{
    GLWindow::windows.remove(this);
    glfwDestroyWindow(window);
}

void GLWindow::swapBuffers()
{
    glfwSwapBuffers(window);
}

void GLWindow::makeContextCurrent()
{
    glfwMakeContextCurrent(window);
}

void GLWindow::setErrorCallback(void (*a)(int, const char *))
{
    glfwSetErrorCallback(a);
}

void GLWindow::getFramebufferSize(int *width, int *height)
{
    glfwGetFramebufferSize(window,width,height);
}

bool GLWindow::getShouldClose()
{
    return glfwWindowShouldClose(window);
}

void GLWindow::setShouldClose(int b)
{
    glfwSetWindowShouldClose(window,b);
}

void GLWindow::windowResizeCallback(glm::ivec2 res)
{

}

void GLWindow::pollEvents()
{
    glfwPollEvents();
}

void GLWindow::waitEvents()
{
    glfwWaitEvents();
}

void GLWindow::setKeyCallback(void (*a)(GLWindow* window,int key,int scancode,int action,int mods)){
    keyCallbackPointer=a;
}
void GLWindow::setCursorCallback(void (*a)(GLWindow *, double, double)){
    cursorCallbackPointer=a;
}
void GLWindow::setMouseButtonCallback(void (*a)(GLWindow *, int, int, int)){
    mouseButtonCallbackPointer=a;
}

void GLWindow::keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods){
    //iterate trough all windows and dispatch the messages
    std::list<GLWindow*>::const_iterator iterator;
    for(iterator=windows.begin();iterator != windows.end();++iterator){
        GLWindow* w = (*iterator);
        if(w->window ==window){
            w->keyCallback(key, scancode, action, mods);
            return;
        }
    }

}

void GLWindow::keyCallback(int key,int scancode,int action,int mods){
//  printf("keyCallback\n");
    if(keyCallbackPointer!=0)
        (*keyCallbackPointer)(this,key,scancode,action,mods);
}

void GLWindow::cursorCallback(GLFWwindow *window, double x, double y){
//    printf("cursorCallback\n");
    std::list<GLWindow*>::const_iterator iterator;
    for(iterator=windows.begin();iterator != windows.end();++iterator){
        GLWindow* w = (*iterator);
        if(w->window ==window){
            w->cursorCallback( x, y);
            return;
        }
    }

}
void GLWindow::cursorCallback(double x, double y){
    if(cursorCallbackPointer!=0){
        (*cursorCallbackPointer)(this,x,y);
    }
}
void GLWindow::mouseButtonCallback(GLFWwindow *window, int button, int action, int mod){
    std::list<GLWindow*>::const_iterator iterator;
    for(iterator=windows.begin();iterator != windows.end();++iterator){
        GLWindow* w = (*iterator);
        if(w->window ==window){
            w->mouseButtonCallback(button,action, mod);
            return;
        }
    }

}
void GLWindow::mouseButtonCallback(int button, int action, int mod){
    if(mouseButtonCallbackPointer!=0){
        (*mouseButtonCallbackPointer)(this,button,action,mod);
    }
}

glm::dvec2 GLWindow::getCursorPosition()
{
    glm::dvec2 pos;
    glfwGetCursorPos(window,&pos.x,&pos.y);
    return pos;
}

void GLWindow::setCursorPosition(glm::dvec2 pos)
{
    glfwSetCursorPos(window,pos.x,pos.y);
}

int GLWindow::getMouseButton(int button)
{
    glfwGetMouseButton(window,button);
}

int GLWindow::getKey(int key)
{
    glfwGetKey(window,key);
}

void GLWindow::setInputMode(int mode, int value)
{
    glfwSetInputMode(window,mode,value);
}


glm::ivec2 tg::GLWindow::getResolution()
{
    int width,height;
    glfwGetFramebufferSize(window,&width,&height);
    return glm::ivec2(width,height);
}

