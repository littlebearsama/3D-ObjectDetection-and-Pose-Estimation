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
#include "PointCloudProjected.h"
#include <stdio.h>

using namespace tg;

PointCloudProjected::PointCloudProjected()
{
  initialized=false;
  this->depthTex=0;
  this->depthChanged=false;

  this->rgbTex=0;
  this->rgbChanged=false;
  this->diffuseShading=true;
  this->f=glm::vec2(600);
  this->uv0=glm::vec2(320,240);
  this->depthScale=1;
  this->pointSize=2;
  this->meanDepth = 0.0;
}

PointCloudProjected::PointCloudProjected(cv::Mat depth, cv::Mat rgb, glm::vec2 f, glm::vec2 uv0, float depthScale)
{
  initialized=false;
  this->programDotted=0;
  this->depthData=depth;
  this->depthTex=0;
  this->depthChanged=true;

  this->rgbData=rgb;
  this->rgbTex=0;
  this->rgbChanged=true;
  this->diffuseShading=true;
  this->f=f;
  this->uv0=uv0;
  this->depthScale = depthScale;
  this->pointSize=2;
}

PointCloudProjected::~PointCloudProjected()
{
  if(depthTex!=0)
  {
    delete depthTex;
    depthTex=0;
  }
  if(rgbTex!=0)
  {
    delete rgbTex;
    rgbTex=0;
  }
  if(programDotted!=0)
  {
    delete programDotted;
    programDotted=0;
  }
}

void PointCloudProjected::initBuffers()
{
  depthTex=new GLTexture2D(depthData);
  depthTex->setFilter(GL_NEAREST,GL_NEAREST);
  glm::ivec2 r(depthData.cols, depthData.rows);

  //create VBO Data:
  std::vector<glm::vec2> positions(r.x*r.y);
  std::vector<glm::vec2> texcoords(r.x*r.y);

  int i=0;
  for(int m=0; m<r.y; m++)
  {
    for(int n=0; n<r.x; n++)
    {
      positions[i] = glm::vec2(float(n),float(m));
      texcoords[i] = glm::vec2(float(n)/r.x,float(m)/r.y);
      i++;
    }
  }

  //upload vbo data:
  glGenVertexArrays(1,&VAO);
  glBindVertexArray(VAO);

  glGenBuffers(1,&posVBO);
  glBindBuffer(GL_ARRAY_BUFFER,posVBO);
  glBufferData(GL_ARRAY_BUFFER,positions.size()*sizeof(glm::vec2),&positions[0],GL_STATIC_DRAW);
  GLuint posLoc = programDotted->getAttribLocation("position");
  glVertexAttribPointer(posLoc,2,GL_FLOAT,GL_FALSE,0,NULL);
  glEnableVertexAttribArray(posLoc);

  glGenBuffers(1,&texcoordVBO);
  glBindBuffer(GL_ARRAY_BUFFER,texcoordVBO);
  glBufferData(GL_ARRAY_BUFFER,texcoords.size()*sizeof(glm::vec2),&texcoords[0],GL_STATIC_DRAW);
  GLuint texCoord = programDotted->getAttribLocation("TextureCoordinates");
  glVertexAttribPointer(texCoord,2,GL_FLOAT,GL_FALSE,0,NULL);
  glEnableVertexAttribArray(texCoord);

  // create ibo data
  glm::ivec2 ri(r.x-1,r.y-1); // it seems like the last 80 rows all have depth 0
  polyCount=6*(ri.x)*(ri.y);
  gridIndexData.resize(polyCount);
  for(int m=0; m<(ri.y); m++){
    for(int n=0; n<(ri.x); n++){
      //create indices:
      gridIndexData[(m*(ri.x)+n)*6+0] =  m    *(r.x) + n;
      gridIndexData[(m*(ri.x)+n)*6+1] =  m    *(r.x) + n+1;
      gridIndexData[(m*(ri.x)+n)*6+2] = (m+1) *(r.x) + n;

      gridIndexData[(m*(ri.x)+n)*6+3] =  m    *(r.x) + n+1;
      gridIndexData[(m*(ri.x)+n)*6+4] = (m+1) *(r.x) + n+1;
      gridIndexData[(m*(ri.x)+n)*6+5] = (m+1) *(r.x) + n;
    }
  }

  //upload ibo data:
  glGenBuffers(1,&IBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,IBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,gridIndexData.size()*sizeof(GLuint),&gridIndexData[0],GL_STATIC_DRAW);

  tg::GLUtils::checkForOpenGLError("[PointCloudProjected::draw]");
  glBindVertexArray(0);
}

void PointCloudProjected::initInContext(Scene *scene)
{
  //load shader
  this->programDotted=new GLSLProgram();
  programDotted->compileShader(std::string(TOMGINE_5_SHADER) + "projPointCloudDotted.fsh");
  programDotted->compileShader(std::string(TOMGINE_5_SHADER) + "projPointCloudDotted.vsh");
  programDotted->link();

  // set uniforms
  programDotted->use();
  programDotted->setUniform("f", f);
  programDotted->setUniform("c", uv0);
  programDotted->setUniform("depthScale",depthScale);
  programDotted->setUniform("pointSize",pointSize);
  programDotted->setUniform("depthTex",0);
  programDotted->setUniform("rgbTex",1);
  programDotted->setUniform("lightDir", glm::vec3(0,0,-1));
  programDotted->setUniform("lightPos", glm::vec3(0,0,1));
  programDotted->setUniform("diffuseLight", glm::vec3(1,1,1));
  programDotted->setUniform("diffuseColor", glm::vec3(1,1,1));

  mvpUniform=programDotted->getUniformLocation("mvp");
  tg::GLUtils::checkForOpenGLError("[PointCloudProjected::initInContext]");

  initialized=true;
}

void PointCloudProjected::removedWhileInContext()
{
  //delete shader if created:
  if(programDotted){
    delete programDotted;
    programDotted=0;
  }
  //delete textures if created:
  if(depthTex){
    delete depthTex;
    depthTex=0;
  }
  if(rgbTex){
    delete rgbTex;
    rgbTex=0;
  }
}

void PointCloudProjected::draw(Scene *scene)
{
  if(initialized)
  {
    if(depthChanged)
    {
      depthMutex.lock();

      if(!depthTex)
        initBuffers();
      else
        depthTex->updateTexture(depthData);

      depthChanged=false;
      depthMutex.unlock();
    } // if(depthChanged)

    if(rgbChanged)
    {
      rgbMutex.lock();

      if(!rgbTex)
        rgbTex=new GLTexture2D(rgbData);
      else
        rgbTex->updateTexture(rgbData);

      rgbChanged=false;
      rgbMutex.unlock();
    } // if(rgbChanged)

    //bind texture vbo and shader:
    //render:
    tg::GLUtils::checkForOpenGLError("[PointCloudProjected::draw]");
    glBindVertexArray(VAO);
    glActiveTexture(GL_TEXTURE0);
    depthTex->bind();
    glActiveTexture(GL_TEXTURE1);
    rgbTex->bind();
    programDotted->use();
    scene->getCam()->applyMat(-1,-1,-1,mvpUniform);

    //    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    programDotted->setUniform("diffuseShading", diffuseShading);

    if(diffuseShading)
    {
      glDrawElements(GL_TRIANGLES,gridIndexData.size(),GL_UNSIGNED_INT,0);//GL_TRIANGLES
    }
    else
    {
      glPointSize(pointSize);
      glDrawArrays(GL_POINTS,0,depthData.rows*depthData.cols);
      glPointSize(1.0f);
    }

    tg::GLUtils::checkForOpenGLError("[PointCloudProjected::draw]");

  } // if(initialized)
}

void PointCloudProjected::updateDepthData(cv::Mat data)
{
  depthMutex.lock();
  this->depthData=data.clone();
  this->depthChanged=true;
  depthMutex.unlock();
}

void PointCloudProjected::updateRGBData(cv::Mat data)
{
  rgbMutex.lock();
  this->rgbData=data.clone();
  this->rgbChanged=true;
  rgbMutex.unlock();

}

void PointCloudProjected::setModelMat(glm::mat4 pos)
{
  this->modelMat = pos;
}
