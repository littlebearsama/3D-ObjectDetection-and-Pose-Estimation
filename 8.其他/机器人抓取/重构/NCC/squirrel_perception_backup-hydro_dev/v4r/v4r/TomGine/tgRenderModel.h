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

#ifndef TG_RENDER_MODEL
#define TG_RENDER_MODEL

#include <stdio.h>
#include <vector>

#include "tgMathlib.h"
#include "tgPose.h"
#include "tgModel.h"
#include "tgMaterial.h"

#include <GL/glu.h>

namespace TomGine {

/** @brief Advanced model for rendering including pose, lighting, material, speed up techniques and bounding primitive (sphere).  */
class tgRenderModel: public tgModel
{
protected:
  GLuint m_displaylist;
  bool m_displaylist_initialized;

  GLuint m_vertexVBO;
  GLuint m_triangleIBO;
  GLuint m_quadIBO;
  std::vector<unsigned> m_triangleIDX;
  std::vector<unsigned> m_quadIDX;
  bool m_bufferobject_initialized;

public:
  /** @brief Defines how the object is rendered. */
  enum RenderMode
  {
    RENDERNORMAL, ///< render object normal (glBegin(), glVertex()).
    DISPLAYLIST, ///< render object using display lists. glCalllist().
    BUFFEROBJECT
    ///< render object using buffer objects.
  };
  tgPose m_pose; ///< The pose of the object (see tgPose).
  tgMaterial m_material; ///< Material of the object (see tgMaterial)

  /** @brief Creates empty model with random material */
  tgRenderModel();
  /** @brief Copy constructor (deletes displaylists and VBOs)*/
  tgRenderModel(const tgRenderModel& model);
  /** @brief Creates model from tgModel with random material. */
  tgRenderModel(const tgModel& model);
  /** @brief Destroys bounding sphere (m_bsmodel), display lists and buffer objects. */
  ~tgRenderModel();

  void destroy();

  void operator=(const tgModel& model);

  /** @brief Set color of this render model */
  void SetColor(float r, float g, float b, float a=1.0f)
  {
    m_material.Color(r, g, b, a);
  }
  void SetColor(int r, int g, int b, int a=255)
  {
    m_material.Color(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f, float(a) / 255.0f);
  }
  void SetColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a=255)
  {
    m_material.Color(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f, float(a) / 255.0f);
  }

  /** @brief computes pose from mean of vertices and updates current pose accordingly */
  void computePose();

  /** @brief Generates a display list from the faces of the model. */
  void GenDisplayList();
  /** @brief Generates a buffer object. */
  void GenBufferObject(GLenum usage=GL_STATIC_DRAW);
  /** @brief Updates the vertices of the buffer object. */
  void UpdateBufferObjectVertices(GLenum usage=GL_STATIC_DRAW);
  void UpdateBufferObjectVertices(const std::vector<size_t>& vertex_indices);
  /** @brief Draws the model as buffer object. */
  void DrawBufferObject(bool normals=true, bool texture=true);

  /** @brief Draw all data in model. */
  virtual void Draw();

  /** @brief Draws the faces of the render model. Applies pose, material and lighting.
   *  @param lighting		Enable/Disable OpenGL lighting calculations.
   *  @param rmode		Rendering mode (NORMAL, DISPLAYLIST, BUFFEROBJECT; see RenderMode) */
  void DrawFaces(bool lighting = true, RenderMode rmode = RENDERNORMAL);

  /** @brief Draws vertex normals of the model.
   *  @param length The length of the normal vector. */
  virtual void DrawNormals(float length);

};

} // namespace TomGine

#endif
