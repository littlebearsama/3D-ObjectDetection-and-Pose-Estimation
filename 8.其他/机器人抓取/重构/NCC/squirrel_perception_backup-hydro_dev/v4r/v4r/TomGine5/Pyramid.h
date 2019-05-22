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

#ifndef _TOMGINE_5_IP_PYRAMID_
#define _TOMGINE_5_IP_PYRAMID_

#include "v4r/TomGine5/GLContext.h"
#include "v4r/TomGine5/GLTexture.h"
#include "v4r/TomGine5/GLSLProgram.h"

namespace tg{


/**
 * @brief The Pyramid class couples itself onto a texture and creates an image pyramid into the mipmaps.
 *        Unlike common mipmaps, the image gets filtered before downscaling.
 */
class Pyramid{
private:
    /*
     * Height of the image pyramid
     */
    int m_depth;

    /*
     * Pointer to the texture the Pyramid is coupled with
     */
    GLTexture2D* m_Texture;

    /*
     * texture size
     */
    int m_width;
    int m_height;

    /*
     * Two textures are needed for vertical and horizontal pass of shader
     */
    GLTexture2D* m_cacheTexture1;
    GLTexture2D* m_cacheTexture2;//this one stores the mipmap
    GLuint m_tex1Framebuffer;
    GLuint m_tex2Framebuffer;
    GLuint m_tex2ReadFramebuffer;


    /*
     * Blur program with according subroutine handles for vertical and horizontal blurring
     */
    /*GLSLProgram m_blur;
    GLuint m_hBlurSub;
    GLuint m_vBlurSub;*/
    GLSLProgram m_blurv;
    GLSLProgram m_blurh;

    GLuint m_quadVAO;
    GLuint m_quadVBO;
    GLuint* m_drawFramebuffers;


public:
    Pyramid(GLTexture2D* base,int depth);
    ~Pyramid();


    /**
     * @brief update: updates the mipmaps of the coupled texture with new blurred images
     */
    void update();
};
}

#endif
