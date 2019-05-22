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
#include "SceneCam.h"
#include "GLWindow.h"
#include <stdio.h>
#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <GLFW/glfw3.h>


using namespace tg;

void SceneCam::applyMat(GLuint modelViewUniform, GLuint normalUniform, GLuint projectionUniform, GLuint MVPUniform)
{
  if(modelViewUniform>=0){
    glUniformMatrix4fv(modelViewUniform,1,GL_FALSE,(GLfloat*)&m_modelView);
  }
  if(normalUniform>=0){
    glUniformMatrix3fv(normalUniform,1,GL_FALSE,(GLfloat*)&m_normal);
  }
  if(projectionUniform>=0){
    glUniformMatrix4fv(projectionUniform,1,GL_FALSE,(GLfloat*)&m_projection);
  }
  if(MVPUniform>=0){

    glUniformMatrix4fv(MVPUniform,1,GL_FALSE,(GLfloat*)&m_MVP);
    //std::cout<< glm::to_string(m_MVP)<<std::endl;
  }
}

void SceneCam::setProjection(glm::ivec2 r, glm::vec2 f, glm::vec2 c, glm::vec2 z)
{
  float fx= 2.0f * f.x / r.x; // normalize e.g. from [0 ... 640] to [0 ... 2]
  float fy = 2.0f * f.y / r.y; // normalizee.g. from [0 ... 480] to [0 ... 2]
  float cx = 1.0 - (2.0f * c.x / r.x); // move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = (2.0f * c.y / r.y) - 1.0f; // flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (z.y + z.x) / (z.x - z.y); // entries for clipping planes
  float z2 = 2.0f * z.y * z.x / (z.x - z.y); // look up for gluPerspective

  // intrinsic matrix
  glm::mat4& p = m_projection;
  p[0][0] = fx; p[1][0] = 0;   p[2][0] = cx;  p[3][0] = 0;
  p[0][1] = 0;  p[1][1] = fy; p[2][1] = cy;  p[3][1] = 0;
  p[0][2] = 0;  p[1][2] = 0;   p[2][2] = z1;  p[3][2] = z2;
  p[0][3] = 0;  p[1][3] = 0;   p[2][3] = -1;    p[3][3] = 0;
}

void SceneCam::setProjection(const glm::mat4& p)
{
  m_projection = p;
}

void SceneCam::setModelView(const glm::mat4& mv)
{
  m_modelView = mv;
}

void SceneCam::print() const
{
  const glm::mat4& p = m_projection;
  const glm::mat4& m = m_modelView;
  printf("projection:\n");
  printf("%f %f %f %f\n", p[0][0], p[1][0], p[2][0], p[3][0]);
  printf("%f %f %f %f\n", p[0][1], p[1][1], p[2][1], p[3][1]);
  printf("%f %f %f %f\n", p[0][2], p[1][2], p[2][2], p[3][2]);
  printf("%f %f %f %f\n", p[0][3], p[1][3], p[2][3], p[3][3]);
  printf("modelview:\n");
  printf("%f %f %f %f\n", m[0][0], m[1][0], m[2][0], m[3][0]);
  printf("%f %f %f %f\n", m[0][1], m[1][1], m[2][1], m[3][1]);
  printf("%f %f %f %f\n", m[0][2], m[1][2], m[2][2], m[3][2]);
  printf("%f %f %f %f\n", m[0][3], m[1][3], m[2][3], m[3][3]);
}


FixedCam::FixedCam(glm::mat4 modelView, glm::mat3 normal, glm::mat4 projection, glm::mat4 MVP)
{
  SceneCam::m_modelView=modelView;
  SceneCam::m_normal=normal;
  SceneCam::m_projection=projection;
  SceneCam::m_MVP=MVP;
}


void tg::FixedCam::set(glm::mat4 modelView, glm::mat3 normal, glm::mat4 projection, glm::mat4 MVP)
{
  SceneCam::m_modelView=modelView;
  SceneCam::m_normal=normal;
  SceneCam::m_projection=projection;
  SceneCam::m_MVP=MVP;
}


FPSCam::FPSCam()
{
  fovv = 60;
  speed = glm::vec3(1.3,1.3,1.0/360.0*3.0*M_PI);
}

FPSCam::~FPSCam()
{

}


void FPSCam::setFovHorizontal(float fov){
  fovv=fov;
}

void FPSCam::rotate(double pitch,double yaw){
  pitchYaw.x+=pitch*(double)speed.z;
  pitchYaw.y+=yaw*(double)speed.z;
}

void FPSCam::move(float forth, float back, float left, float right){

  float forward= (forth-back)*speed.x;
  float leftward= (left-right)*speed.y;
  glm::vec4 movement(leftward,0,forward,1);
  glm::mat4 i;
  glm::mat4 r1 = glm::rotate(i, -(float)pitchYaw.x, glm::vec3(-1.0f, 0.0f, 0.0f));
  glm::mat4 r2 = glm::rotate(i,-(float)pitchYaw.y,glm::vec3(0,-1,0));

  glm::vec4 dx= r2*r1*movement;

  pos+= glm::vec3(dx.x,dx.y,dx.z);

}

glm::mat4 FPSCam::getTransformM(int width,int height){
  glm::mat4 p = glm::perspective((float)fovv*(float)M_PI/180.0f, (float)width/(float)height, 0.01f, 100.f);

  glm::mat4 i;
  glm::mat4 r1 = glm::rotate(i, (float)pitchYaw.x, glm::vec3(-1.0f, 0.0f, 0.0f));
  glm::mat4 r2 = glm::rotate(i,(float)pitchYaw.y,glm::vec3(0,-1,0));
  glm::mat4 r = r1*r2;

  glm::mat4 t = glm::translate(i, pos);


  return p*r*t;
}

void FPSCam::printPosition(){
  printf("pitch=%f yaw=%f   pos: x=%f y=%f z=%f\n",(float)pitchYaw.x,(float)pitchYaw.y,pos.x,pos.y,pos.z);

}
void FPSCam::setPosition(glm::vec2 pitch_yaw, glm::vec3 pos){
  pitchYaw=glm::dvec2(pitch_yaw.x,pitch_yaw.y);
  this->pos=pos;
}



void FPSCam::update(GLWindow *window)
{
  //calc dt:

  float dt=0.016;
  float up=0,left=0,down=0,right=0;
  if(window->getKey(GLFW_KEY_W)){
    //printf("test\n");
    up=1.0*dt;
  }
  if(window->getKey(GLFW_KEY_A)){
    left=1.0*dt;
  }
  if(window->getKey(GLFW_KEY_S)){
    down=1.0*dt;
  }
  if(window->getKey(GLFW_KEY_D)){
    right=1.0*dt;
  }

  move(up, down, left, right);
  glm::ivec2 res  = window->getResolution();
  SceneCam::m_MVP = getTransformM(res.x,res.y);
}

void FPSCam::keyCallback(GLWindow *window, int key, int scancode, int action, int mods)
{

}

void FPSCam::cursorCallback(GLWindow *window, double x, double y)
{
  if( window->getMouseButton(GLFW_MOUSE_BUTTON_1)){


    glm::dvec2 pos = window->getCursorPosition();
    if(curPosReset){
      cursorPos=pos;
      curPosReset=false;
    }
    glm::dvec2 dpos=cursorPos - pos;
    cursorPos=pos;
    //printf("curpos %lf, %lf \n",cursorPos.x,cursorPos.y);
    rotate(dpos.y, dpos.x);
  }
}

void FPSCam::mouseButtonCallback(GLWindow *window, int button, int action, int mod)
{
  if((button == GLFW_MOUSE_BUTTON_1) && (action ==GLFW_PRESS)){
    //hide cursor if left mouse button is pressed:
    window->setInputMode(GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    curPosReset =true;
  }else{
    //show it otherwise
    window->setInputMode(GLFW_CURSOR, GLFW_CURSOR_NORMAL);

  }
}

OrbitCam::OrbitCam()
{
  m_projection = glm::perspective<float>(45.0f*M_PI/180.0f, 4.0f/3.0f, 0.1f, 10.0f);
  m_modelView = glm::mat4(1.0f);
  m_modelView0 = m_modelView;
  //  m_modelView = glm::translate(glm::mat4(1.0f), glm::vec3(0,0,-1.0f));
  m_cor = glm::vec3(0,0,0);
}

void OrbitCam::SetCOR(const glm::vec3& cor)
{
  m_cor = cor;
}

void OrbitCam::SetExtrinsic(const glm::mat4& E)
{
  m_modelView = E;
  m_modelView0 = m_modelView;
}

void OrbitCam::SetIntrinsic(const glm::mat4& I)
{
  m_projection = I;
}

void OrbitCam::SetNearFar(const float& near, const float& far)
{
  float &z1 = m_projection[2][2];
  float &z2 = m_projection[2][3];

  z1 = (far+near) / (near-far);
  z2 = (2.0*far*near) / (near-far);
}

float OrbitCam::GetFar() const
{
  const float &z1 = m_projection[2][2];
  const float &z2 = m_projection[2][3];

  return (z2 / (z1 + 1.0f));
}

float OrbitCam::GetNear() const
{
  const float &z1 = m_projection[2][2];
  const float &z2 = m_projection[2][3];

  float far = z2 / (z1 + 1.0f);
  return (z2 * far / (z2 - 2.0f * far));
}

glm::vec3 OrbitCam::GetForward() const
{
  return glm::vec3(m_modelView[0][2], m_modelView[1][2], m_modelView[2][2]);
}

glm::vec3 OrbitCam::GetSideward() const
{
  return glm::vec3(m_modelView[0][0], m_modelView[1][0], m_modelView[2][0]);
}

glm::vec3 OrbitCam::GetUpward() const
{
  return glm::vec3(m_modelView[0][1], m_modelView[1][1], m_modelView[2][1]);
}

void OrbitCam::TranslateForward(float d)
{
  glm::vec3 f(m_modelView[0][2], m_modelView[1][2], m_modelView[2][2]);
  m_modelView = glm::translate(m_modelView, f*d);
}

void OrbitCam::TranslateSideward(float d)
{
  glm::vec3 s(m_modelView[0][0], m_modelView[1][0], m_modelView[2][0]);
  m_modelView = glm::translate(m_modelView, s*d);
}

void OrbitCam::TranslateUpward(float d)
{
  glm::vec3 u(m_modelView[0][1], m_modelView[1][1], m_modelView[2][1]);
  m_modelView = glm::translate(m_modelView, u*d);
}

void OrbitCam::Orbit(glm::vec3 point, glm::vec3 axis, float angle)
{
  m_modelView = glm::translate(m_modelView, point);
  m_modelView = glm::rotate(m_modelView, -angle, axis);
  m_modelView = glm::translate(m_modelView, -point);
}

void OrbitCam::update(GLWindow *window)
{
  //    //calc dt:

  //    float dt=0.016;
  //    float up=0,left=0,down=0,right=0;
  //    if(window->getKey(GLFW_KEY_W)){
  //        //printf("test\n");
  //        up=1.0*dt;
  //    }
  //    if(window->getKey(GLFW_KEY_A)){
  //        left=1.0*dt;
  //    }
  //    if(window->getKey(GLFW_KEY_S)){
  //        down=1.0*dt;
  //    }
  //    if(window->getKey(GLFW_KEY_D)){
  //        right=1.0*dt;
  //    }

  //    move(up, down, left, right);
  //    glm::ivec2 res  = window->getResolution();
  //    SceneCam::m_MVP = getTransformM(res.x,res.y);
  SceneCam::m_MVP = m_projection * m_modelView;
}

void OrbitCam::keyCallback(GLWindow *window, int key, int scancode, int action, int mods)
{
  if(key == GLFW_KEY_Y && action == GLFW_PRESS) // Y==Z in german keyboard layout
    m_modelView = m_modelView0;
}

void OrbitCam::cursorCallback(GLWindow *window, double x, double y)
{
  //    if( window->getMoseButton(GLFW_MOUSE_BUTTON_1)); //{


  //        glm::dvec2 pos = window->getCursorPosition();
  //        if(curPosReset){
  //            cursorPos=pos;
  //            curPosReset=false;
  //        }
  //        glm::dvec2 dpos=cursorPos - pos;
  //        cursorPos=pos;
  //        //printf("curpos %lf, %lf \n",cursorPos.x,cursorPos.y);
  //        rotate(dpos.y, dpos.x);
  //    }


  if (window->getMouseButton(GLFW_MOUSE_BUTTON_1) ||
      window->getMouseButton(GLFW_MOUSE_BUTTON_2) ||
      window->getMouseButton(GLFW_MOUSE_BUTTON_3))
  {
    glm::dvec2 pos = window->getCursorPosition();
    if(curPosReset)
    {
      cursorPos = pos;
      curPosReset = false;
    }
    glm::dvec2 dpos = cursorPos - pos;
    cursorPos=pos;



    float far = GetFar();
    float near = GetNear();

    if (window->getMouseButton(GLFW_MOUSE_BUTTON_1))
    {
      // rotate
      Orbit(m_cor, GetUpward(), 0.02f * dpos.x);
      Orbit(m_cor, GetSideward(), 0.02f * dpos.y);
    }
    else if ( window->getMouseButton(GLFW_MOUSE_BUTTON_2) )
    {
      // move in x,y direction
      TranslateSideward(0.0001f * (near - far) * dpos.x);
      TranslateUpward(-0.0001f * (near - far) * dpos.y);
    }
    else if ( window->getMouseButton(GLFW_MOUSE_BUTTON_3) )
    {
      // move in z direction
      TranslateForward(0.001f * (near - far) * dpos.x);
      TranslateForward(0.001f * (near - far) * dpos.y);
    }

  }
}

void OrbitCam::mouseButtonCallback(GLWindow *window, int button, int action, int mod)
{
  if((button == GLFW_MOUSE_BUTTON_1 ||
      button == GLFW_MOUSE_BUTTON_2 ||
      button == GLFW_MOUSE_BUTTON_3) &&
     (action ==GLFW_PRESS))
  {
    //hide cursor if left mouse button is pressed:
    //          window->setInputMode(GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    curPosReset =true;
  }
  //  else{
  //    //show it otherwise
  //    window->setInputMode(GLFW_CURSOR, GLFW_CURSOR_NORMAL);

  //  }
}
