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

#include "tgTextureModel.h"
#include "tgError.h"

using namespace TomGine;

void tgTextureModel::clearGLTextures()
{
  for (unsigned i = 0; i < m_tex_gl.size(); i++)
    delete (m_tex_gl[i]);
  m_tex_gl.clear();
}

void tgTextureModel::syncTextures()
{
  clearGLTextures();

  for (unsigned i = 0; i < m_tex_cv.size(); i++)
  {
    tgTexture2D *tex = new tgTexture2D();

    if (!m_tex_cv[i].empty())
    {
      if (!m_tex_cv[i].isContinuous())
        printf("[tgTextureModel::syncTextures] Warning, data of texture not memory aligned\n");

      tex->SetTexEnvMode(GL_MODULATE);
      tex->Load(m_tex_cv[i].data, m_tex_cv[i].cols, m_tex_cv[i].rows, GL_RGB, GL_BGR, GL_UNSIGNED_BYTE);
    }

    m_tex_gl.push_back(tex);
  }
  sync = true;
}

tgTextureModel::tgTextureModel() :
  tgRenderModel::tgRenderModel(), sync(false)
{
}

tgTextureModel::tgTextureModel(const tgTextureModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
  if (!m.m_tex_gl.empty())
  {
    printf("[tgTextureModel::tgTextureModel(const tgTextureModel &)] "
      "Warning, OpenGL Textures cannot be copied in tgTextureModel::m_tex_gl (%lu) '%s'\n", m.m_tex_gl.size(),
        m.name.c_str());
  }

  this->m_tex_cv.assign(m.m_tex_cv.size(), cv::Mat3b());
  for (unsigned i = 0; i < m.m_tex_cv.size(); i++)
    m.m_tex_cv[i].copyTo(this->m_tex_cv[i]);

  this->m_face_tex_id = m.m_face_tex_id;
}

tgTextureModel::tgTextureModel(const tgRenderModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
}

tgTextureModel::tgTextureModel(const tgModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
}

tgTextureModel::~tgTextureModel()
{
  clearGLTextures();
}

tgTextureModel&
tgTextureModel::operator=(const tgTextureModel &m)
{
  tgRenderModel::operator=(m);
  this->sync = false;

  if (!m.m_tex_gl.empty())
    printf("[tgTextureModel::operator=(const tgTextureModel &)] "
      "Warning, cannot copy tgTexture2D in tgTextureModel::m_tex_gl (%lu)\n", m.m_tex_gl.size());

  this->m_tex_cv.assign(m.m_tex_cv.size(), cv::Mat3b());
  for (unsigned i = 0; i < m.m_tex_cv.size(); i++)
    m.m_tex_cv[i].copyTo(this->m_tex_cv[i]);

  this->m_face_tex_id = m.m_face_tex_id;

  return *this;
}

void tgTextureModel::Clear()
{
  tgModel::Clear();
  m_tex_cv.clear();
  m_face_tex_id.clear();
  sync = false;
}

void tgTextureModel::Draw(bool textured, RenderMode rmode)
{
  DrawFaces(textured, rmode);

  m_pose.Activate();
  DrawLines();
  DrawPoints();
  DrawColorPoints();
  m_pose.Deactivate();
}

void tgTextureModel::DrawFacesInt(bool textured) const
{
  m_material.Activate();
  glLineWidth(m_line_width);

  for (size_t i = 0; i < m_faces.size(); i++)
  {

    if (textured)
      m_tex_gl[m_face_tex_id[i]]->Bind();

    const tgFace &f = m_faces[i];
    if (f.v.size() == 3)
      glBegin(GL_TRIANGLES);
    else if (f.v.size() == 4)
      glBegin(GL_QUADS);
    else
    {
      printf("[tgTextureModel::DrawFacesInt()] Warning, no suitable face format\n");
      printf("[tgTextureModel::DrawFacesInt()] Face has %d vertices (supported: 3 or 4)\n", (int) f.v.size());
      glEnd();
      continue;
    }

    for (size_t j = 0; j < f.v.size(); j++)
    {
      const tgVertex& v = m_vertices[f.v[j]];
      glTexCoord2f(v.texCoord.x, v.texCoord.y);
      glNormal3f(v.normal.x, v.normal.y, v.normal.z);
      glVertex3f(v.pos.x, v.pos.y, v.pos.z);
    }
    glEnd();
  }

  glDisable(GL_TEXTURE_2D);
}

void tgTextureModel::DrawFaces(bool textured, RenderMode rmode)
{

  if (textured)
    if ((!sync) || (m_tex_cv.size() != m_tex_gl.size()))
      syncTextures();

  if (textured && sync && m_face_tex_id.size() == m_faces.size())
  {
    m_pose.Activate();
    if (rmode == DISPLAYLIST)
    {
      if (m_displaylist_initialized)
      {
        glCallList(m_displaylist);
      } else
      {
        m_displaylist = glGenLists(1);
        glNewList(m_displaylist, GL_COMPILE_AND_EXECUTE);
        DrawFacesInt(textured);
        glEndList();
        m_displaylist_initialized = true;
      }
    } else
    {
      DrawFacesInt(textured);
    }
    m_pose.Deactivate();
  } else
  {
    tgRenderModel::DrawFaces(true, rmode);
  }
}

void tgTextureModel::TextureFromImage(const tgCamera& cam, const cv::Mat3b& image)
{
  ComputeFaceNormals();
  m_face_tex_id.clear();

  std::vector<bool> vertex_used(m_vertices.size(), false);
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    TomGine::tgFace &f = m_faces[i];

    for (size_t j = 0; j < f.v.size(); j++)
    {
      const unsigned &vidx = f.v[j];
      if (vertex_used[vidx])
        continue;

      vertex_used[vidx] = true;
      TomGine::tgVertex &v = m_vertices[vidx];

      v.texCoord = cam.GetTexCoords(v.pos);
      v.texCoord.y = 1.0f - v.texCoord.y; // cv::Mat to OpenGL
    }
    m_face_tex_id.push_back(0);
  }
  m_tex_cv.resize(1);
  image.copyTo(m_tex_cv[0]);
}

void tgTextureModel::OptimizeTextures()
{
  std::vector<bool> vertex_updated(m_vertices.size(), false);
  for (size_t i = 0; i < m_tex_cv.size(); i++)
  {
    cv::Mat3b &tex = m_tex_cv[i];

    // collect vertices belonging to that tex
    std::vector<TomGine::tgVertex*> vertices;
    for (size_t j = 0; j < m_face_tex_id.size(); j++)
    {
      if (m_face_tex_id[j] == i)
      {
        for (size_t k = 0; k < m_faces[j].v.size(); k++)
        {
          const unsigned &vidx = m_faces[j].v[k];
          if (!vertex_updated[vidx])
            vertices.push_back(&m_vertices[vidx]);
          vertex_updated[vidx] = true;
        }
      }
    }

    if (vertices.empty())
    {
      tex = cv::Mat3b(); // clear data of unused textures // todo remove tex and re-map m_face_tex_id
      continue;
    }

    vec2 tc_min(FLT_MAX, FLT_MAX);
    vec2 tc_max(0.0f, 0.0f);
    int width = tex.cols;
    int height = tex.rows;
    float dwidth = 1.0f / width;
    float dheight = 1.0f / height;

    // get bounding box of texture coordinates
    for (unsigned j = 0; j < vertices.size(); j++)
    {
      const vec2 &tc = vertices[j]->texCoord;

      if (tc.x < tc_min.x)
        tc_min.x = tc.x;
      if (tc.y < tc_min.y)
        tc_min.y = tc.y;
      if (tc.x > tc_max.x)
        tc_max.x = tc.x;
      if (tc.y > tc_max.y)
        tc_max.y = tc.y;
    }

    // if tex is already optimal, leave it
    if (tc_min.x < dwidth && tc_min.y < dheight && tc_max.x > (1.0f - dwidth) && tc_max.y > (1.0f - dheight))
      return;

    // bounding box in pixel coordinates
    int i_min_x = int(floor(tc_min.x * width));
    int i_min_y = int(floor(tc_min.y * height));
    int i_max_x = int(ceil(tc_max.x * width));
    int i_max_y = int(ceil(tc_max.y * height));
    int i_width = i_max_x - i_min_x;
    int i_height = i_max_y - i_min_y;

    // adjust texture width and height to be divisible by 8 (otherwise OpenGL Textures cause problems)
    if (i_width % 8)
      i_width = (1 + i_width / 8) * 8;
    if (i_min_x + i_width >= width)
      i_min_x -= (i_min_x + i_width - width);
    if (i_height % 8)
      i_height = (1 + i_height / 8) * 8;
    if (i_min_y + i_height >= height)
      i_min_y -= (i_min_y + i_height - height);

    if (i_min_x < 0)
      i_min_x = 0;
    if (i_min_y < 0)
      i_min_y = 0;
    if (i_width > width)
      i_width = width;
    if (i_height > height)
      i_height = height;

    // copy sub texture
    cv::Rect rect(i_min_x, i_min_y, i_width, i_height);

    float r_width = float(width) / i_width;
    float r_height = float(height) / i_height;

    cv::Mat3b tex_tmp(tex, rect);
    tex_tmp.copyTo(tex);

    // adjust texture coordinats
    float d_min_x = r_width * dwidth * i_min_x;
    float d_min_y = r_height * dheight * i_min_y;

    for (unsigned j = 0; j < vertices.size(); j++)
    {
      vec2 &tc = vertices[j]->texCoord;
      tc.x = tc.x * r_width - d_min_x;
      tc.y = tc.y * r_height - d_min_y;
    }

  }
}
