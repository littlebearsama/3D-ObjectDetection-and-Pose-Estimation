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


#ifndef _SCENE_H_
#define _SCENE_H_
#include "SceneCam.h"
#include "GLSLProgram.h"
#include <list>
namespace tg {
class GLContext;
class GLTexture2D;
class Scene;
class SceneObject{
private:
    bool m_visible;
public:
    //besides the constructor OpenGL resources should be initialized inside the active
    //OpenGL context
    virtual void initInContext(Scene* scene) = 0;
    virtual void removedWhileInContext() = 0;

    //the object has the opportunity to draw itself
    virtual void draw(Scene* scene) = 0;

    //a currently unused feature
    void setVisibility(bool visible){ m_visible = visible;}

};
class Scene{
protected:
    std::list<SceneObject*> m_objects;
    SceneCam* m_cam;
    glm::vec4 m_clearColor;
    bool m_clear;
    GLSLProgram* m_shadeDiffuse;

public:
    Scene();
    virtual ~Scene(){}
    virtual void setCam(SceneCam* cam){m_cam=cam;}
    SceneCam* getCam(){return m_cam;}
    virtual void addSceneObject(SceneObject* object);
    virtual void removeSceneObject(SceneObject* object);
    //virtual glm::ivec2 getResolution(){return glm::ivec2();}
    //void keyCallback(int key,int scancode, int action, int mods);
    virtual void draw()=0;
    //virtual glm::ivec2 getResolution() = 0;
    virtual bool flipYAxis(){return false;}

    GLSLProgram* GetShaderDiffuse() { return m_shadeDiffuse; }

    void setClearColor(GLfloat red,GLfloat green, GLfloat blue, GLfloat alpha){m_clearColor=glm::vec4(red,green,blue,alpha);}
    void setClear(bool clear){m_clear = clear;}

};
class OffscreenScene : public Scene{
private:
    GLTexture2D* m_texture;
    GLContext* m_context;
    GLuint m_framebuffer;
    GLuint m_depth;

public:
    OffscreenScene(int width,int height,GLContext* context);
    ~OffscreenScene();

//    void addSceneObject(SceneObject* object);
//    void removeSceneObject(SceneObject* object);

    void draw();
    glm::ivec2 getResolution();
    GLTexture2D* getTexture(){return m_texture;}



};
}
#endif
