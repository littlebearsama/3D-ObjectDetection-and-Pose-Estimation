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

#ifndef __GLTEXTURE__
#define __GLTEXTURE__

#define GLM_FORCE_RADIANS
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace tg{



/*
  TODO:
load texture from file.
Reload texture with different sizes
glTexStorage2D ...... predefines and fixes the size of the texture.
  */
/**
 * @brief The GLTexture2D class handles an OpenGL 2D texture, and handles the conversion of opencv Mat matrix
 * to a texture.
 */
class GLTexture2D{
private:
    GLuint m_glTextureID;
    int m_width;
    int m_height;
    int* m_widths;
    int* m_heights;
    int m_nrMipmaps;
    bool m_mipmaps;
    cv::Point2d* m_mipmapRes;
    int m_cvFormat;
    GLenum m_glType;
    GLenum m_glFormat;
    GLenum m_glInternal;
    void constructor(int width,int height,GLint glInternalFormat,GLenum glFormat,GLenum glType,void* data=0);
    void constructor(int width,int height,int cvType,void* data=0);

public:
    /**
     * @brief GLTexture2D Constructor accepts the pure Data and creates a new texture.
     * @param width
     * @param height
     * @param cvType: OpenCV format like CV_32FC2
     * @param data
     */
    GLTexture2D(int width,int height,int cvType,void* data=0);
    GLTexture2D(int width,int height,GLint glInternalFormat,GLenum glFormat,GLenum glType,void* data=0);
    GLTexture2D(cv::Mat data,bool bgrToRgb=false);
    ~GLTexture2D();



    void setFilter(GLint glFilterType);
    void setFilter(GLint glMinFilterType,GLint glMagFilterType);
    void bind();
    /** @brief	Shows texture as image via cv::imshow. Extremely slow! */
    void imshow(const char* title,GLint texLevel=0);
    GLuint getHandle();


    cv::Point2i getRes(GLint texLevel=0);


    //todo: might handle conversion from BGR to RGB
    /** @brief Uploads data to the GPU.*/
    void updateTexture(cv::Mat data);
    /** @brief Uploads data to the GPU.*/
    void updateTexture(void* data);

    //todo: might handle conversion from BGR to RGB
    /** @brief Downloads the texture from GPU. */
    cv::Mat getData(GLint texLevel=0);


    void getData(void* data,GLint texLevel=0);

    /** @brief	Generates mipmaps for texture. */
    void generateMipmaps();

    int getCVType();



    /**
     * @brief convertCVToGLType
     * @param cvType
     * @return
     */
    static GLenum convertCVToGLType(int cvType);

    /**
     * @brief convertCVToGLFormat
     * @param cvType
     * @return
     */
    static GLenum convertCVToGLFormat(int cvType);

    /**
     * @brief convertCVToGLInternalFormat
     * @param cvType
     * @return
     */
    static GLenum convertCVToGLInternalFormat(int cvType);
};
};

#endif
