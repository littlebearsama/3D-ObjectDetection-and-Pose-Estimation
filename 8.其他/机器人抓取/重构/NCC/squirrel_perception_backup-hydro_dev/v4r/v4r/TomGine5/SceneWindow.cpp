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
#include "SceneWindow.h"
#include "stdio.h"

using namespace tg;




SceneWindow::SceneWindow(GLuint width, GLuint height, std::string title, tg::GLWindow *share)
    : GLWindow(width,height,title,share), Scene()
{

}

void SceneWindow::draw()
{
    makeContextCurrent();

    if(Scene::m_cam){
        Scene::m_cam->update(this);
    }
    glEnable(GL_DEPTH_TEST);
    //glDisable(GL_DEPTH_TEST);
    glm::ivec2 res = getResolution();
    glViewport(0,0,res.x,res.y);
    //glViewport(0,0,640,480);
    if(Scene::m_clear){
        glm::vec4 c = Scene::m_clearColor;
//        glClearColor(c.x,c.y,c.z,c.w);
        glClearColor(0.5f,0.5f,0.5f,c.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //fflush(stdout);

    }
    std::list<SceneObject*>::const_iterator iterator;
    for (iterator = Scene::m_objects.begin(); iterator != Scene::m_objects.end(); ++iterator) {
        (*iterator)->draw(this);
    }

    glBindVertexArray(0);

}

/*glm::ivec2 SceneWindow::getResolution()
{
    int width,height;
    glfwGetFramebufferSize(window,&width,&height);
    return glm::ivec2(width,height);
}*/

void SceneWindow::keyCallback(int key, int scancode, int action, int mods)
{
    GLWindow::keyCallback(key,scancode,action,mods);
    if(Scene::m_cam){
        Scene::m_cam->keyCallback(this,key,scancode,action,mods);
    }

}

void SceneWindow::cursorCallback(double x, double y)
{
    GLWindow::cursorCallback(x,y);
    if(Scene::m_cam){
        Scene::m_cam->cursorCallback(this,x,y);
    }

}

void SceneWindow::mouseButtonCallback(int button, int action, int mod)
{
    GLWindow::mouseButtonCallback(button,action,mod);
    if(Scene::m_cam){
        Scene::m_cam->mouseButtonCallback(this,button,action,mod);
    }
}
