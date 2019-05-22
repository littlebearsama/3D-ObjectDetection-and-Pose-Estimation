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

#ifndef TG_TEXTURE_MODEL
#define TG_TEXTURE_MODEL

#include "tgRenderModel.h"
#include "tgTexture.h"
#include "tgCamera.h"
#include <opencv2/core/core.hpp>

namespace TomGine {


class tgTextureModel: public tgRenderModel
{
public: //todo PRIVATE !!!
  bool sync;
  std::vector<tgTexture2D*> m_tex_gl;
  void syncTextures();
  void clearGLTextures();

public:
  std::vector<cv::Mat3b> m_tex_cv;          ///< Textures of the model as cv::Mat3b
  std::vector<unsigned> m_face_tex_id;      ///< index of tgTexture2D in m_textures for each face in tgModel::m_face

  /** @brief constructors and copy-constructors */
  tgTextureModel();
  tgTextureModel(const tgTextureModel &m);
  tgTextureModel(const tgRenderModel &m);
  tgTextureModel(const tgModel &m);
  virtual ~tgTextureModel();

  tgTextureModel& operator=(const tgTextureModel &m);

  virtual void Clear();

  /** @brief Calls DrawTexFaces if a texture is available, otherwise use tgRenderModel::DrawFaces */
  virtual void Draw(bool textured = true, RenderMode rmode = RENDERNORMAL);

  /** @brief Draws faces textured (without pose activation) **/
  virtual void DrawFacesInt(bool textured) const;

  /** @brief Synchronizes textures with OpenGL if necessary and draws textured faces. */
  virtual void DrawFaces(bool textured = true, RenderMode rmode = RENDERNORMAL);

  /** @brief Allows to force synchronization of textures with OpenGL */
  inline void Sync(){ sync = false; }

  /** @brief Project model using camera and assign texture coordinates */
  void TextureFromImage(const tgCamera& cam, const cv::Mat3b& image);

  /** @brief Crop textures to minimum area necessary
   *  @brief Adjust tex coordinates according to new tex size */
  void OptimizeTextures();

};

}

#endif
