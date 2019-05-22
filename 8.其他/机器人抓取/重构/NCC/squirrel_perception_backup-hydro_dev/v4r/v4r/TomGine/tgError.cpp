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

#include "tgError.h"

namespace TomGine{

GLenum tgCheckError(std::string pre_msg, bool detailed)
{
  GLenum error = glGetError();

  switch(error){
  case GL_NO_ERROR:
    // No error has been recorded.
    // The value of this symbolic constant is guaranteed to be 0.
    break;
  case GL_INVALID_ENUM:
    printf("%s OpenGL Error: GL_INVALID_ENUM\n", pre_msg.c_str());
    if(detailed){
      printf("  An unacceptable value is specified for an enumerated argument.\n");
      printf("  The offending command is ignored\n");
      printf("  and has no other side effect than to set the error flag.\n");
    }
    break;
  case GL_INVALID_VALUE:
    printf("%s OpenGL Error: GL_INVALID_VALUE\n", pre_msg.c_str());
    if(detailed){
      printf("  A numeric argument is out of range.\n");
      printf("  The offending command is ignored\n");
      printf("  and has no other side effect than to set the error flag.\n");
    }
    break;
  case GL_INVALID_OPERATION:
    printf("%s OpenGL Error: GL_INVALID_OPERATION\n", pre_msg.c_str());
    if(detailed){
      printf("  The specified operation is not allowed in the current state.\n");
      printf("  The offending command is ignored\n");
      printf("  and has no other side effect than to set the error flag.\n");
    }
    break;
  case GL_INVALID_FRAMEBUFFER_OPERATION:
    printf("%s OpenGL Error: GL_INVALID_FRAMEBUFFER_OPERATION\n", pre_msg.c_str());
    if(detailed){
      printf("  The framebuffer object is not complete. The offending command\n");
      printf("  is ignored and has no other side effect than to set the error flag.\n");
    }
    break;
  case GL_OUT_OF_MEMORY:
    printf("%s OpenGL Error: GL_OUT_OF_MEMORY\n", pre_msg.c_str());
    if(detailed){
      printf("  There is not enough memory left to execute the command.\n");
      printf("  The state of the GL is undefined, except for the state of the error flags,\n");
      printf("  after this error is recorded.\n");
    }
    break;
  default:
    printf("%s OpenGL Error: unknown\n", pre_msg.c_str());
    if(detailed)
      printf("  No detailed description available.\n");
    break;
  }
  return error;
}

GLenum tgCheckFBError(GLenum target, std::string pre_msg){

  GLenum error = glCheckFramebufferStatus(target);

  switch(error){
  case GL_FRAMEBUFFER_COMPLETE:
    break;
  case GL_FRAMEBUFFER_UNDEFINED:
    // is returned if target is the default framebuffer, but the default framebuffer does not exist.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_UNDEFINED\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
    // is returned if any of the framebuffer attachment points are framebuffer incomplete.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
    // is returned if the framebuffer does not have at least one image attached to it.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
    // is returned if the value of GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE
    // is GL_NONE for any color attachment point(s) named by GL_DRAWBUFFERi.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
    // is returned if the value of GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE
    // is GL_NONE for any color attachment point(s) named by GL_DRAWBUFFERi.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_UNSUPPORTED:
    // is returned if the combination of internal formats of the attached images violates
    // an implementation-dependent set of restrictions.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_UNSUPPORTED\n", pre_msg.c_str());
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
    // is returned if the value of GL_RENDERBUFFER_SAMPLES is not the same
    // for all attached renderbuffers; if the value of GL_TEXTURE_SAMPLES is the not same for all attached textures;
    // or, if the attached images are a mix of renderbuffers and textures, the value of GL_RENDERBUFFER_SAMPLES
    // does not match the value of GL_TEXTURE_SAMPLES.
    printf("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE\n", pre_msg.c_str());
    break;
  default:
    printf("%s OpenGL FBO Error: unknown\n", pre_msg.c_str());
    break;
  }

  return error;
}

} // namespace TomGine

