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

#ifndef TG_FRAMEBUFFEROBJECT
#define TG_FRAMEBUFFEROBJECT

#include "tgTexture.h"

namespace TomGine {

/** @brief Class for managing an OpenGL frame buffer object.
 *
 *  For the usage of frame buffer objects please refer to http://www.opengl.org/wiki/Framebuffer_Object */
class tgFrameBufferObject
{
private:
  unsigned m_width, m_height; ///< width and height of the frame buffer object

  GLuint m_fbo_id; ///< ID of the OpenGL frame buffer object.

public:
  std::vector<tgTexture2D*> texColor; ///< GPU texture storage for the color buffer.
  tgTexture2D* texDepth; ///< GPU texture storage for the depth buffer.

  /** @brief Create the frame buffer object on the GPU using OpenGL.
   * 	@param w,h		The size (width, height) of the frame buffer object in pixel.
   * 	@param colorInternal	The internal format for the color texture used as color buffer (texColor).
   * 	@param depthInternal	The internal format for the depth texture used as depth buffer (texDepth). */
  tgFrameBufferObject(unsigned w, unsigned h,
                      GLint colorInternal = GL_RGBA,
                      GLint depthInternal = GL_DEPTH_COMPONENT);
  tgFrameBufferObject(unsigned w, unsigned h,
                      std::vector<GLint>& colorInternal, // order corresponds to GL_COLOR_ATTACHMENT[i]
                      GLint depthInternal = GL_DEPTH_COMPONENT);
  /** @brief Destroy the frame buffer object on the GPU. */
  ~tgFrameBufferObject();

  /** @brief Activates the frame buffer object. OpenGL renders to the texture storage and not to the screen. */
  void Bind();
  /** @brief Deactivates the frame buffer object. OpenGL renders to the screen and not to the texture storage. */
  void Unbind();

//  /** @brief Save color buffer (texColor) to a file. */
//  void SaveColor(const char* filename);
//  /** @brief Save depth buffer (texDepth) to a file. */
//  void SaveDepth(const char* filename);

};

}

#endif
