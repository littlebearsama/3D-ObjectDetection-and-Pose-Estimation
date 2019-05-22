/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
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
 * @author thomas.moerwald
 *
 */
#include "tgFrameBufferObject.h"
#include "tgError.h"
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>

using namespace TomGine;

tgFrameBufferObject::tgFrameBufferObject(unsigned w, unsigned h,
                                         GLint colorInternal, GLint depthInternal)
{

  m_width = w;
  m_height = h;

  texColor.assign(1, new tgTexture2D());
  texDepth = new tgTexture2D();

  texColor[0]->Bind();
  texColor[0]->Load(NULL, m_width, m_height, colorInternal, GL_RGBA, GL_UNSIGNED_BYTE);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] fbo_tex");

  texDepth->Bind();
  texDepth->Load(NULL, m_width, m_height, depthInternal, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] fbo_depth_tex");

  glGenFramebuffers(1, &m_fbo_id);
  Bind();

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColor[0]->GetTextureID(), 0);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach color texture");

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth->GetTextureID(), 0);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach depth texture");

  Unbind();

  if (tgCheckFBError(GL_FRAMEBUFFER, "[tgFrameBufferObject::tgFrameBufferObject]") != GL_FRAMEBUFFER_COMPLETE
      || tgCheckError("[tgFrameBufferObject::tgFrameBufferObject]") != GL_NO_ERROR)
  {
    std::string errmsg =
        std::string("[tgFrameBufferObject::tgFrameBufferObject] Error generating frame buffer objects");
    throw std::runtime_error(errmsg.c_str());
  }
  glDisable(GL_TEXTURE_2D);
}

tgFrameBufferObject::tgFrameBufferObject(unsigned w, unsigned h,
                                         std::vector<GLint>& colorInternal, GLint depthInternal)
{

  m_width = w;
  m_height = h;

  for(size_t i=0; i<colorInternal.size(); i++)
  {
    texColor.push_back(new tgTexture2D());
    texColor[i]->Bind();
    texColor[i]->Load(NULL, m_width, m_height, colorInternal[i], GL_RGBA, GL_UNSIGNED_BYTE);
  }
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] fbo_tex");

  texDepth = new tgTexture2D();
  texDepth->Bind();
  texDepth->Load(NULL, m_width, m_height, depthInternal, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] fbo_depth_tex");

  glGenFramebuffers(1, &m_fbo_id);
  Bind();

  std::vector<GLenum> drawBuffers;
  for(size_t i=0; i<texColor.size(); i++)
  {
    drawBuffers.push_back(GL_COLOR_ATTACHMENT0+i);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0+i, GL_TEXTURE_2D, texColor[i]->GetTextureID(), 0);
  }
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach color texture");

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth->GetTextureID(), 0);
  tgCheckError("[tgFrameBufferObject::tgFrameBufferObject] attach depth texture");

  glDrawBuffers(drawBuffers.size(), &drawBuffers[0]);

  Unbind();

  if (tgCheckFBError(GL_FRAMEBUFFER, "[tgFrameBufferObject::tgFrameBufferObject]") != GL_FRAMEBUFFER_COMPLETE
      || tgCheckError("[tgFrameBufferObject::tgFrameBufferObject]") != GL_NO_ERROR)
  {
    std::string errmsg =
        std::string("[tgFrameBufferObject::tgFrameBufferObject] Error generating frame buffer objects");
    throw std::runtime_error(errmsg.c_str());
  }
  glDisable(GL_TEXTURE_2D);
}

tgFrameBufferObject::~tgFrameBufferObject()
{
  if (glIsFramebuffer(m_fbo_id))
    glDeleteFramebuffers(1, &m_fbo_id);

  for(size_t i=0; i<texColor.size(); i++)
    delete texColor[i];
  delete texDepth;
}

void tgFrameBufferObject::Bind()
{
  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo_id);
  tgCheckError("[tgFrameBufferObject::Bind]");
}

void tgFrameBufferObject::Unbind()
{
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

//void tgFrameBufferObject::SaveColor(const char* filename)
//{
//  texColor[0]->Bind();
//  cv::Mat img(m_height, m_width, CV_8UC3);
//  glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
//  glDisable(GL_TEXTURE_2D);
//  tgCheckError("[tgFrameBufferObject::SaveColor]");
//  cv::imwrite(filename, img);
//}

//void tgFrameBufferObject::SaveDepth(const char* filename)
//{
//  texDepth->Bind();
//  cv::Mat img(m_height, m_width, CV_8U);
//  glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, img.data);
//  glDisable(GL_TEXTURE_2D);
//  tgCheckError("[tgFrameBufferObject::SaveDepth]");
//  cv::imwrite(filename, img);
//}
