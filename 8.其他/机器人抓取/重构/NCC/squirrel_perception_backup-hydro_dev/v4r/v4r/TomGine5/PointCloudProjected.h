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
#ifndef _POINT_CLOUD_PROJECTED_
#define _POINT_CLOUD_PROJECTED_

#include "Scene.h"
#include "GLTexture.h"
#include "GLSLProgram.h"
#include <boost/thread.hpp>
namespace tg{


class PointCloudProjected : public SceneObject{
private:


    bool initialized;
    GLuint VAO;
    GLuint posVBO;
    GLuint texcoordVBO;
    GLuint IBO;
    GLSLProgram* programDotted;
    //there should be several uniforms for matrices and stuff
    GLuint mvpUniform;//uniform of modelview projection matrix

    GLuint polyCount;
    std::vector<GLuint> gridIndexData;

    boost::mutex depthMutex;
    cv::Mat depthData;
    GLTexture2D* depthTex;
    bool depthChanged;

    boost::mutex rgbMutex;
    cv::Mat rgbData;
    GLTexture2D* rgbTex;
    bool rgbChanged;
    glm::mat4 transform;

    bool diffuseShading;
    glm::vec2 f;
    glm::vec2 uv0;
    float depthScale;
    glm::mat4 modelMat;
    float pointSize;
    double meanDepth;

    void initBuffers();

public:

    PointCloudProjected();
    PointCloudProjected(cv::Mat depth,cv::Mat rgb,glm::vec2 f,glm::vec2 uv0,float depthScale=1);
    /**
      * I hate to say it, but the destructor has to wait for removedWhileInContext to be finished
      */
    ~PointCloudProjected();
    virtual void initInContext(Scene* scene);
    virtual void removedWhileInContext();

    //the object has the opportunity to draw itself
    virtual void draw(Scene* scene);

    void updateDepthData(cv::Mat data);

    void updateRGBData(cv::Mat data);
    void setPointSize(float size);
    void setModelMat(glm::mat4 pos);

    double getMeanDepth(){ return meanDepth; }

    void setDiffuse(bool v) { diffuseShading=v; }

};
}
#endif
