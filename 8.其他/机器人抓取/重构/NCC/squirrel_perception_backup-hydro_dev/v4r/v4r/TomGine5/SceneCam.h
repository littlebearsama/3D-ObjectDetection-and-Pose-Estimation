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
 * @author simon.schreiberhuber, thomas.moerwald
 *
 */

#ifndef _SCENE_CAM_H_
#define _SCENE_CAM_H_
#include "GLUtils.h"
#include <glm/glm.hpp>
namespace tg{

class GLWindow;

/*
 * TODO: The camera class needs a new structure for proper lightning
 */
class SceneCam{
protected:
    glm::mat4 m_modelView;
    glm::mat3 m_normal;
    glm::mat4 m_projection;
    glm::mat4 m_MVP;
public:
    virtual void calcMat(glm::ivec2 res){}

    virtual void applyMat(GLuint modelViewUniform,GLuint normalUniform,GLuint projectionUniform,GLuint MVPUniform);
    void setProjection(glm::ivec2 res, glm::vec2 f, glm::vec2 c, glm::vec2 zrange);
    void setProjection(const glm::mat4& p);
    void setModelView(const glm::mat4& mv);
    void print() const;

    const glm::mat4& getModelView() const { return m_modelView; }
    const glm::mat3& getNormalMatrix() const { return m_normal; }
    const glm::mat4& getProjectionMatrix() const { return m_projection; }
    const glm::mat4& getMVP() const { return m_MVP; }

    virtual void update(GLWindow* window){}

    virtual void keyCallback(GLWindow* window,int key,int scancode,int action,int mods){}
    virtual void cursorCallback(GLWindow* window,double x,double y){}
    virtual void mouseButtonCallback(GLWindow* window,int button, int action, int mod){}



};


/**
 * @brief The FixedCam class returns the user defined camera matrices
 */
class FixedCam : public SceneCam{
private:
public:
    FixedCam(glm::mat4 modelView,glm::mat3 normal,glm::mat4 projection,glm::mat4 MVP);
    void set(glm::mat4 modelView,glm::mat3 normal,glm::mat4 projection,glm::mat4 MVP);
    //void calcMat();
    //void applyMat(GLuint modelViewUniform,GLuint normalUniform,GLuint projectionUniform,GLuint MVPUniform);
};

class FPSCam : public SceneCam{
private:

    //vectors that store camera orientation and position
    glm::dvec2 pitchYaw;
    glm::vec3 pos;

    //vector stores speed in forward sideward direction and turnrate
    glm::vec3 speed;

    //vertical field of view
    float fovv;

    bool curPosReset;
    glm::dvec2 cursorPos;

public:

    FPSCam();
    ~FPSCam();
    //virtual void calcMat(glm::ivec2 res);
    //virtual void applyMat(GLuint modelViewUniform,GLuint normalUniform,GLuint projectionUniform,GLuint MVPUniform);


    /*
     * All these methods are recycled from a long time ago:
     * Don't look to close at them!
     */
    //Method to change fov
    void setFovHorizontal(float fov);

    //method to rotate around pitch and yaw axis
    void rotate(double pitch,double yaw);
    //move the camera
    void move(float forth,float back,float left,float right);

    //get back the transformation matrix
    glm::mat4 getTransformM(int width,int height);

    void printPosition();
    void setPosition(glm::vec2 pitch_yaw,glm::vec3 pos);



    virtual void update(GLWindow* window);

    virtual void keyCallback(GLWindow* window,int key,int scancode,int action,int mods);
    virtual void cursorCallback(GLWindow* window,double x,double y);
    virtual void mouseButtonCallback(GLWindow* window,int button, int action, int mod);
};

/**
 * @brief The OrbitCam class is supposed to orbit around one object
 * @author thomas.moerwald
 */
class OrbitCam : public SceneCam{

private:
  glm::vec3 m_cor; // center of rotation

  bool curPosReset;
  glm::dvec2 cursorPos;
  glm::mat4 m_modelView0;

public:
  OrbitCam();

  void SetCOR(const glm::vec3& cor);

  void SetExtrinsic(const glm::mat4& E);
  void SetIntrinsic(const glm::mat4& I);

  void SetNearFar(const float& near, const float& far);
  float GetFar() const;
  float GetNear() const;
  glm::vec3 GetForward() const;
  glm::vec3 GetSideward() const;
  glm::vec3 GetUpward() const;

  void TranslateForward(float d);
  void TranslateSideward(float d);
  void TranslateUpward(float d);
  void Orbit(glm::vec3 point, glm::vec3 axis, float angle);

  virtual void update(GLWindow* window);

  virtual void keyCallback(GLWindow* window,int key,int scancode,int action,int mods);
  virtual void cursorCallback(GLWindow* window,double x,double y);
  virtual void mouseButtonCallback(GLWindow* window,int button, int action, int mod);

};
}
#endif
