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
#include "Scene.h"
#include "GLTexture.h"
#include "GLContext.h"
using namespace tg;

Scene::Scene(){
    m_cam=0;
    m_clear=1;
    m_clearColor= glm::vec4(0);

    // init shader
    m_shadeDiffuse = new GLSLProgram();
    m_shadeDiffuse->compileShader(std::string(TOMGINE_5_SHADER) + "diffuse.fsh");
    m_shadeDiffuse->compileShader(std::string(TOMGINE_5_SHADER) + "diffuse.vsh");
    m_shadeDiffuse->link();
    tg::GLUtils::checkForOpenGLError("[Scene::Scene] init m_shadeDiffuse");
}


void Scene::addSceneObject(SceneObject *object)
{
    object->initInContext(this);
    m_objects.push_back(object);

}

void Scene::removeSceneObject(SceneObject *object)
{
    object->removedWhileInContext();
    m_objects.remove(object);
}


OffscreenScene::OffscreenScene(int width, int height, GLContext *context){
    m_context=context;
    m_context->makeCurrent();
    m_texture= new GLTexture2D(width,height,CV_32FC4);

    glGenRenderbuffers(1, &m_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);


    glGenFramebuffers(1,&m_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER,m_framebuffer);
    m_texture->bind();
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,m_texture->getHandle(),0);

}

OffscreenScene::~OffscreenScene()
{
    delete m_texture;
    glDeleteFramebuffers(1,&m_framebuffer);
}

/*void OffscreenScene::addSceneObject(SceneObject *object)
{

}

void OffscreenScene::removeSceneObject(SceneObject *object)
{

}*/

void OffscreenScene::draw(){

    m_context->makeCurrent();
    glBindFramebuffer(GL_FRAMEBUFFER,m_framebuffer);
    glEnable(GL_DEPTH_TEST);
    cv::Point2i res= m_texture->getRes();
    glViewport(0,0,res.x,res.y);
    if(Scene::m_clear){
        glm::vec4 c = Scene::m_clearColor;
        glClearColor(c.x,c.y,c.z,c.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    std::list<SceneObject*>::const_iterator iterator;
    for (iterator = Scene::m_objects.begin(); iterator != Scene::m_objects.end(); ++iterator) {
        (*iterator)->draw(this);
    }
    glBindVertexArray(0);

}

glm::ivec2 OffscreenScene::getResolution()
{
    return glm::ivec2(m_texture->getRes().x,m_texture->getRes().y);
}


