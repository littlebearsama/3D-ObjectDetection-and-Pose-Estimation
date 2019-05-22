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

#include "Pyramid.h"
#include <stdio.h>


using namespace tg;


Pyramid::Pyramid(GLTexture2D *base, int depth){


    m_depth=depth;
    //glGenVertexArrays(1, &homographicReadVAO);
   // glBindVertexArray(homographicReadVAO);

    /*
      Generate VertexArrayObject
      */
    glm::vec2 quadData[]={glm::vec2(0,0),glm::vec2(0,1),glm::vec2(1,1),glm::vec2(1,0)};
    glGenBuffers(1,&m_quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(quadData)*sizeof(glm::vec2),quadData,GL_STATIC_DRAW);



    glGenVertexArrays(1,&m_quadVAO);
    glBindVertexArray(m_quadVAO);

    /*
      Load blur shader:
      */

   /* m_blur.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlur.fsh");
    m_blur.bindFragDataLocation(0,"fragColor");

    m_blur.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlur.vsh");

    m_blur.link();
    m_blur.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    m_blur.use();
    m_blur.setUniform("tex",0);
    m_vBlurSub = glGetSubroutineIndex(m_blur.getHandle(),GL_FRAGMENT_SHADER,"blurv");
    m_hBlurSub = glGetSubroutineIndex(m_blur.getHandle(),GL_FRAGMENT_SHADER,"blurh");
    */
    m_blurv.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlurv.fsh");
    m_blurv.bindFragDataLocation(0,"fragColor");

    m_blurv.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlur.vsh");

    m_blurv.link();
    m_blurv.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    m_blurv.use();
    m_blurv.setUniform("tex",0);

    m_blurh.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlurh.fsh");
    m_blurh.bindFragDataLocation(0,"fragColor");

    m_blurh.compileShader(std::string(TOMGINE_5_SHADER) + "pyrBlur.vsh");

    m_blurh.link();
    m_blurh.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    m_blurh.use();
    m_blurh.setUniform("tex",0);





    /*
      create textures:
      */
    m_Texture=base;
    base->setFilter(GL_LINEAR,GL_LINEAR_MIPMAP_LINEAR);
    base->generateMipmaps();//generates mipmaps

    m_width=m_Texture->getRes().x;
    m_height=m_Texture->getRes().y;
    m_cacheTexture1 = new tg::GLTexture2D(m_width,m_height,m_Texture->getCVType());
    m_cacheTexture2 = new tg::GLTexture2D(m_width,m_height,m_Texture->getCVType());





    /*
      Generate Framebuffers:
      */

    glGenFramebuffers(1,&m_tex1Framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER,m_tex1Framebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,m_cacheTexture1->getHandle(),0);

    glGenFramebuffers(1,&m_tex2Framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER,m_tex2Framebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,m_cacheTexture2->getHandle(),0);

    glGenFramebuffers(1,&m_tex2ReadFramebuffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER,m_tex2ReadFramebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_READ_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,m_cacheTexture2->getHandle(),0);

    m_drawFramebuffers = new GLuint[depth];
    glGenFramebuffers(depth,m_drawFramebuffers);

    /*
     * Generate draw framebuffers to the different mipmap levels:
     */
    for (int i=0;i<depth;i++){
        //printf("i:%d\n",i);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER,m_drawFramebuffers[i]);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        glFramebufferTexture(GL_DRAW_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,m_Texture->getHandle(),i+1);
    }





    tg::GLUtils::checkForOpenGLError("[tgPyramid::constructor]");



    update();

}

Pyramid::~Pyramid(){
    glDeleteFramebuffers(m_depth,m_drawFramebuffers);
    glDeleteFramebuffers(1,&m_tex1Framebuffer);
    glDeleteFramebuffers(1,&m_tex2Framebuffer);
    glDeleteFramebuffers(1,&m_tex2ReadFramebuffer);
    glDeleteBuffers(1,&m_quadVBO);
    glDeleteVertexArrays(1,&m_quadVAO);
    delete m_cacheTexture1;
    delete m_cacheTexture2;


}

void Pyramid::update(){
    float scale=1;
    glActiveTexture(GL_TEXTURE0);
    //step trough the pyramid:
    for (int i=0;i<m_depth;i++){

        m_Texture->bind();
        glBindFramebuffer(GL_FRAMEBUFFER,m_tex1Framebuffer);
        glViewport(0,0,m_width*scale,m_height*scale);
        m_blurv.use();
        m_blurv.setUniform("scale",1.0f);
        m_blurv.setUniform("lod",(float)i);

        tg::GLUtils::checkForOpenGLError("[tg::GLTexture::bind]");

        glBindVertexArray(m_quadVAO);
        glDrawArrays(GL_TRIANGLE_FAN,0,4);



        //m_cacheTexture1->imshow("texture1");


        m_cacheTexture1->bind();
        glBindFramebuffer(GL_FRAMEBUFFER,m_tex2Framebuffer);
        glViewport(0,0,m_width*scale,m_height*scale);
        m_blurh.use();

        m_blurh.setUniform("scale",scale);
        //m_blurh.setUniform("lod",(float)i);
        glDrawArrays(GL_TRIANGLE_FAN,0,4);

        glBindFramebuffer(GL_READ_FRAMEBUFFER,m_tex2ReadFramebuffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER,m_drawFramebuffers[i]);


        //m_cacheTexture2->imshow("texture2");

        /*
         * Download and scale the blurred image to the according mipmap
         */
        glBlitFramebuffer(0,0,m_width*scale,m_height*scale,0,0,m_width*scale/2,m_height*scale/2,GL_COLOR_BUFFER_BIT,GL_LINEAR);

        scale=scale/2;
    }
    glBindVertexArray(0);
}

