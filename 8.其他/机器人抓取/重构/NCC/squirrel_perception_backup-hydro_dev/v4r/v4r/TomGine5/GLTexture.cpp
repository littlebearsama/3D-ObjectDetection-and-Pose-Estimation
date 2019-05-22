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

#include "GLTexture.h"
#include "GLUtils.h"
//#include "GLError.h"
#include <stdio.h>
#include <cmath>
#include <opencv/cv.h>


using namespace tg;
/** @brief	Returns the opengl datatype of a opencv mat format. */
GLenum GLTexture2D::convertCVToGLType(int cvType){
    GLenum glType=-1;
    int depth=CV_MAT_DEPTH(cvType);
    //printf("depth%d\n",(int)depth);

    switch (depth){
    case CV_8U:
        glType=GL_UNSIGNED_BYTE;
        break;
    case CV_8S:
        glType=GL_BYTE;
        break;
    case CV_16U:
        glType=GL_UNSIGNED_SHORT;
        break;
    case CV_16S:
        glType=GL_SHORT;
        break;
    case CV_32S:
        glType=GL_INT;
        break;
    case CV_32F:
        glType=GL_FLOAT;
        break;
    case CV_64F:
        glType=-1;
        printf("error!!TODO");
        break;
    default:
        printf("Error: Unsupported format");
        return -1;
    }

    return glType;
}

/** @brief	Returns the opengl format of a opencv mat format. */
GLenum GLTexture2D::convertCVToGLFormat(int cvType){
    GLenum glFormat=-1;
    int format=(cvType>>CV_CN_SHIFT)+1;
    //printf("format%d\n",(int)format);
    switch (format){
    case 1:
        glFormat=GL_RED;
        //printf("red\n");
        break;
    case 2:
        glFormat=GL_RG;
        //printf("rg\n");
        break;
    case 3:
        glFormat=GL_RGB;
        //printf("rgb\n");
        break;
    case 4:
        glFormat=GL_RGBA;
        //printf("rgba\n");
        break;
    default:
        printf("Error: Unsupported format");
        return -1;
    }
    return glFormat;
}
/** @brief	Converts a opencv format into a opengl internal format. */
GLenum GLTexture2D::convertCVToGLInternalFormat(int cvType){
    const GLenum formats[4][6]={{GL_R8   ,GL_R8,GL_R16      ,GL_R16    ,GL_R32I   ,GL_R32F},
                                {GL_RG8  ,GL_RG8,GL_RG16    ,GL_RG16   ,GL_RG32I  ,GL_RG32F},
                                {GL_RGB8 ,GL_RGB8,GL_RGB16  ,GL_RGB16  ,GL_RGB32I ,GL_RGB32F},
                                {GL_RGBA8,GL_RGBA8,GL_RGBA16,GL_RGBA16 ,GL_RGBA32I,GL_RGBA32F}
                               };
    int channels,depth;
    GLenum type = convertCVToGLType(cvType);
    switch (type){
    case GL_UNSIGNED_BYTE:
        depth=0;
        break;
    case GL_BYTE:
        depth=1;
        break;
    case GL_UNSIGNED_SHORT:
        depth=2;
        break;
    case GL_SHORT:
        depth=3;
        break;
    case GL_UNSIGNED_INT:
        depth=4;
        break;
    case GL_FLOAT:
        depth=5;
        break;
    default:
        printf("Error: Unsupported format");
        return -1;
    }

    GLenum format = convertCVToGLFormat(cvType);
    switch (format){
    case GL_RED:
        channels=0;
        break;
    case GL_RG:
        channels=1;
        break;
    case GL_RGB:
        channels=2;
        break;
    case GL_RGBA:
        channels=3;
        break;
    default:
        printf("Error: to many channels");
        return -1;
    }





    return formats[channels][depth];
}

int convertGLtoCVFormat(GLenum glFormat,GLenum glType){
    int channels=0;
    switch (glFormat){
    case GL_RED:
        //printf("1 channel\n");
        channels=1;
        break;
    case GL_RG:
        channels=2;
        break;
    case GL_RGB:
        channels=3;
        break;
    case GL_RGBA:
        channels=4;
        break;

    default:
        printf("Error: Unsupported format");
        return -1;
    }
    int cvType;
    switch (glType){
    case GL_UNSIGNED_BYTE:
        //printf("CV_8U\n");
        cvType=CV_8U;
        break;
    case GL_BYTE:
        //printf("CV_8S\n");
        cvType=CV_8S;
        break;
    case GL_UNSIGNED_SHORT:
        cvType=CV_16U;
        break;
    case GL_SHORT:
        cvType=CV_16S;
        break;
    case GL_UNSIGNED_INT:
        cvType=CV_32S;
        break;
    case GL_FLOAT:
        cvType=CV_32F;
        break;
    default:
        printf("Error: Unsupported format");
        return -1;
    }
    return CV_MAKETYPE(cvType,channels);
}

void GLTexture2D::constructor(int width, int height, GLint glInternalFormat, GLenum glFormat, GLenum glType, void *data){
    glGenTextures(1,&m_glTextureID);
    glGetError();
    glBindTexture(GL_TEXTURE_2D, m_glTextureID);
    glGetError();
    glTexImage2D(GL_TEXTURE_2D, 0,glInternalFormat, width, height, 0,glFormat, glType, data);


    //http://open.gl/textures
    // filtering.
    //TODO: http://www.opengl.org/wiki/Common_Mistakes#Creating_a_complete_texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float color[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    tg::GLUtils::checkForOpenGLError("[tg::GLTexture::constructor]");

    m_glType=glType;
    m_glFormat=glFormat;
    m_glInternal=glInternalFormat;

    m_width=width;
    m_height=height;
    m_nrMipmaps=0;

    m_cvFormat=convertGLtoCVFormat(glFormat,glType);
}

void GLTexture2D::constructor(int width, int height, int cvType, void *data){
    GLenum internal= convertCVToGLInternalFormat(cvType);
    GLenum format  = convertCVToGLFormat(cvType);
    GLenum type    = convertCVToGLType(cvType);

    constructor(width,height,internal,format,type,data);

}

GLTexture2D::GLTexture2D(int width, int height, int cvType, void *data){
    GLenum internal= convertCVToGLInternalFormat(cvType);
    GLenum format  = convertCVToGLFormat(cvType);
    GLenum type    = convertCVToGLType(cvType);

    constructor(width,height,internal,format,type,data);

}


GLTexture2D::GLTexture2D(int width, int height, GLint glInternalFormat, GLenum glFormat, GLenum glType, void *data){
    constructor(width,height,glInternalFormat,glFormat,glType,data);
}

GLTexture2D::GLTexture2D(cv::Mat data, bool bgrToRgb){
    if(bgrToRgb){
        cv::Mat m;
        cv::cvtColor(data,m,CV_BGR2RGB);
        //data.convertTo(m,CV_BGR2RGB);
        constructor(data.cols,data.rows,data.type(),m.data);
    }else{
        constructor(data.cols,data.rows,data.type(),data.data);

    }
}

GLTexture2D::~GLTexture2D(){
    glDeleteTextures(1,&m_glTextureID);
    tg::GLUtils::checkForOpenGLError("[tg::GLTexture::destructor]");

    //if mipmaps were created. Delete the storage for mipmap resolutions.
    if(m_nrMipmaps){
        delete[] m_heights;
        delete[] m_widths;
    }
}

void GLTexture2D::setFilter(GLint glFilterType){
    setFilter(glFilterType,glFilterType);
}

void GLTexture2D::setFilter(GLint glMinFilterType, GLint glMagFilterType){
    bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, glMinFilterType);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, glMagFilterType);
}


void GLTexture2D::bind(){

    glBindTexture(GL_TEXTURE_2D,m_glTextureID);
    tg::GLUtils::checkForOpenGLError("[tg::GLTexture::bind]");
}

void GLTexture2D::imshow(const char *title, GLint texLevel){
    cv::Mat im=getData(texLevel);
    if(im.channels()==2){
        //CV_MAT_DEPTH(cvType);
        cv::Mat im2;
        im.convertTo(im2,CV_32FC3);
        cv::imshow(title,im2);
    }else{
        if(im.type()==CV_8U){
            printf("Why you no work!!!!\n");
        }
        cv::imshow(title,im);
    }
}

GLuint GLTexture2D::getHandle(){
    return m_glTextureID;
}

cv::Point2i GLTexture2D::getRes(GLint texLevel){
    if(texLevel==0)
        return cv::Point2d(m_width,m_height);

    if (texLevel<m_nrMipmaps){
        return cv::Point2d(m_widths[texLevel-1],m_heights[texLevel-1]);
    }
    return cv::Point(-1,-1);//TODO: proper error message. Invalid mipmap level.
}

void GLTexture2D::updateTexture(cv::Mat data){
    //TODO: maybe test if the new texture data fits to the old one.
    //printf("TEEEST!!! width %d, height %d \n",m_width, m_height);
    /*if(data.type()==CV_8U){
        printf("richtiger CV type\n");
    }*/
    updateTexture(data.data);
}
void GLTexture2D::updateTexture(void *data){


    //glTexImage2D(GL_TEXTURE_2D,0,m_glInternal,m_width,m_height,0,m_glFormat,m_glType,data);
    bind();
    /*printf("width%d,height%d",m_width,m_height);
    int width,height;
    glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_WIDTH,&width);

    glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_HEIGHT,&height);
    printf("widthReal%d,heightreal%d\n",width,height);
    if(m_glFormat==GL_RED){
        printf("richtiges GLformat!!!\n");
    }
    if(m_glType == GL_UNSIGNED_BYTE){
        printf("richtiger GLtyp\n");
    }
    cv::Mat dataread=getData();
    if(dataread.type()==CV_8U){
        printf("rightCV datatype read data\n");
    }

    printf("dataread width%d height%d\n",dataread.cols,dataread.rows);
    cv::imshow("texData",dataread);*/

    glTexSubImage2D(GL_TEXTURE_2D,0,0,0,m_width,m_height,m_glFormat,m_glType,data);
    glGetError();
}


cv::Mat GLTexture2D::getData(GLint texLevel){


    bind();
    cv::Point2i res = getRes(texLevel);
    cv::Mat im(res.y,res.x,m_cvFormat);
    /*
    int height;
    glGetTexLevelParameteriv(GL_TEXTURE_2D,texLevel,GL_TEXTURE_HEIGHT,&height);

    printf("height original %d, heightviaGL %d \n",m_height, height);
    int width;
    glGetTexLevelParameteriv(GL_TEXTURE_2D,texLevel,GL_TEXTURE_WIDTH,&width);

    printf("width original %d, widthviaGL %d \n",m_width, width);
   // printf("resx %d, resy %d \n",res.x, res.y);

    //printf("width %d, height %d \n",m_width, m_height);

    if(m_glFormat==GL_RED && m_glType==GL_UNSIGNED_BYTE){
        printf("no prob!!\n");
    }
    fflush(stdout);*/
    //for debug sake:
    //glGetTexImage(GL_TEXTURE_2D,texLevel,GL_BLUE,m_glType,im.data);
    glGetTexImage(GL_TEXTURE_2D,texLevel,m_glFormat,m_glType,im.data);
    tg::GLUtils::checkForOpenGLError("[tg::GLTexture::getData]");
    return im;
}
void GLTexture2D::getData(void *data, GLint texLevel){
    //if(m_glFormat==GL_RED)
    //for debug sake
    //cv::Point2i res = getRes(texLevel);
    glGetTexImage(GL_TEXTURE_2D,texLevel,m_glFormat,m_glType,data);
}


void GLTexture2D::generateMipmaps(){
    glActiveTexture(GL_TEXTURE0);
    bind();
    glGenerateMipmap(GL_TEXTURE_2D);
    tg::GLUtils::checkForOpenGLError("[tg::GLTexture::generateMipmaps]");

    /*
    *Store the size of the mipmaps on the client side. People say it is slow to
    *use the glGetTexLevelParameteriv function.
    */
    if (!m_nrMipmaps){
        int nrMipmaps = log2(std::max(m_width,m_height));//std::ceil((float)std::max(m_width,m_height));
        //printf("nr mipmaps: %d,\n",nrMipmaps);
        m_widths=new int[nrMipmaps];
        m_heights=new int[nrMipmaps];
        for (int i=0;i<nrMipmaps;i++){
            glGetTexLevelParameteriv(GL_TEXTURE_2D,i+1,GL_TEXTURE_WIDTH,&m_widths[i]);
            glGetTexLevelParameteriv(GL_TEXTURE_2D,i+1,GL_TEXTURE_HEIGHT,&m_heights[i]);
            //printf("width %d, height %d \n",m_widths[i], m_heights[i]);
        }
        m_nrMipmaps=nrMipmaps;
    }


}

int GLTexture2D::getCVType(){
    return m_cvFormat;
}

