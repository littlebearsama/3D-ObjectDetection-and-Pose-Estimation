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

#include "tgRenderModel.h"
#include "tgShapeCreator.h"
#include "tgError.h"
#include <GL/gl.h>

using namespace TomGine;

tgRenderModel::tgRenderModel() :
  m_displaylist_initialized(false), m_bufferobject_initialized(false)
{
  m_material.Random();
}

tgRenderModel::tgRenderModel(const tgRenderModel& model) :
  m_displaylist_initialized(false), m_bufferobject_initialized(false)
{
  tgModel::operator=(model);

  m_pose = model.m_pose;
  m_material = model.m_material;
}

tgRenderModel::tgRenderModel(const tgModel& model) :
  tgModel(model), m_displaylist_initialized(false), m_bufferobject_initialized(false)
{
  m_material.Random();
}

tgRenderModel::~tgRenderModel()
{
  destroy();
}

void tgRenderModel::destroy()
{
  if (m_displaylist_initialized)
  {
    if(glIsList(m_displaylist))
      glDeleteLists(m_displaylist, 1);
    m_displaylist_initialized = false;
  }

  if (m_bufferobject_initialized)
  {
    if(glIsBuffer(m_vertexVBO))
      glDeleteBuffers(1, &m_vertexVBO);
    if(glIsBuffer(m_triangleIBO))
      glDeleteBuffers(1, &m_triangleIBO);
    if(glIsBuffer(m_quadIBO))
      glDeleteBuffers(1, &m_quadIBO);
    m_bufferobject_initialized = false;
  }
}

void tgRenderModel::operator=(const tgModel& model)
{
  tgModel::operator=(model);

  destroy();
}

void tgRenderModel::GenDisplayList()
{
  if (m_displaylist_initialized) {
    glDeleteLists(m_displaylist, 1);
    m_displaylist_initialized = false;
  }

  m_displaylist = glGenLists(1);

  glNewList(m_displaylist, GL_COMPILE_AND_EXECUTE);
  tgModel::DrawFaces();
  glEndList();

  m_displaylist_initialized = true;
}

void tgRenderModel::GenBufferObject(GLenum usage)
{
  if (m_bufferobject_initialized) {
    if(glIsBuffer(m_vertexVBO))
      glDeleteBuffers(1, &m_vertexVBO);
    if(glIsBuffer(m_triangleIBO))
      glDeleteBuffers(1, &m_triangleIBO);
    if(glIsBuffer(m_quadIBO))
      glDeleteBuffers(1, &m_quadIBO);
    m_bufferobject_initialized = false;
  }

  // collect data
  m_triangleIDX.clear();
  m_quadIDX.clear();
  for (unsigned i = 0; i < m_faces.size(); i++) {
    if (m_faces[i].v.size() == 3) {
      m_triangleIDX.push_back(m_faces[i].v[0]);
      m_triangleIDX.push_back(m_faces[i].v[1]);
      m_triangleIDX.push_back(m_faces[i].v[2]);
    } else if (m_faces[i].v.size() == 4) {
      m_quadIDX.push_back(m_faces[i].v[0]);
      m_quadIDX.push_back(m_faces[i].v[1]);
      m_quadIDX.push_back(m_faces[i].v[2]);
      m_quadIDX.push_back(m_faces[i].v[3]);
    } else {
      printf("[tgRenderModel::GenBufferObject] Warning, no suitable face format\n");
      printf("[tgRenderModel::GenBufferObject] Face has %d vertices (supported: 3 or 4)\n", (int) m_faces[i].v.size());
      return;
    }
  }

  // generate VBOs
  glGenBuffers(1, &m_vertexVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(tgVertex) * m_vertices.size(), &m_vertices[0].pos.x, usage);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate vertex buffer");
#endif
  glGenBuffers(1, &m_triangleIBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * m_triangleIDX.size(), &m_triangleIDX[0], usage);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate triangle buffer");
#endif
  glGenBuffers(1, &m_quadIBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadIBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * m_quadIDX.size(), &m_quadIDX[0], usage);
#ifdef DEBUG
  tgCheckError("[tgRenderModel::GenBufferObject] generate quad buffer");
#endif

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  m_bufferobject_initialized = true;
}

void tgRenderModel::UpdateBufferObjectVertices(GLenum usage)
{
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(tgVertex) * m_vertices.size(), &m_vertices[0], usage);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void tgRenderModel::UpdateBufferObjectVertices(const std::vector<size_t>& vertex_indices)
{
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  tgCheckError("[tgRenderModel::UpdateBufferObjectVertices] A");


  for(size_t i=0; i<vertex_indices.size(); i++)
  {
    const size_t& vi = vertex_indices[i];

    if(vi<m_vertices.size())
    {
      const tgVertex& v = m_vertices[vi];
      glBufferSubData(GL_ARRAY_BUFFER, sizeof(tgVertex)*vi, sizeof(tgVertex), &v);

      if(tgCheckError("[tgRenderModel::UpdateBufferObjectVertices] B")!=GL_NO_ERROR)
      {
        GLint s;
        glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &s);
        printf("[tgRenderModel::UpdateBufferObjectVertices] %lu / %d\n", sizeof(tgVertex)*vi, s);
        return;
      }

    }else
    {
      printf("[tgRenderModel::UpdateBufferObjectVertices] Warning, vertex does not exists (index out of bounds)\n");
    }
  }



  glBindBuffer(GL_ARRAY_BUFFER, 0);
  tgCheckError("[tgRenderModel::UpdateBufferObjectVertices] C");
}

#define BUFFER_OFFSET(i) ((char *)NULL + (i))
void tgRenderModel::DrawBufferObject(bool normals, bool texture)
{

  GLsizei vsize = sizeof(tgVertex);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_INDEX_ARRAY);
  if(normals)
    glEnableClientState(GL_NORMAL_ARRAY);
  if(texture)
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);


  glBindBuffer(GL_ARRAY_BUFFER, m_vertexVBO);
  glVertexPointer(3, GL_FLOAT, vsize, BUFFER_OFFSET(0));
  if(normals)
    glNormalPointer(GL_FLOAT, vsize, BUFFER_OFFSET(12));
  if(texture)
    glTexCoordPointer(2, GL_FLOAT, vsize, BUFFER_OFFSET(24));

  if(!m_triangleIDX.empty())
  {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIBO);
    glDrawElements(GL_TRIANGLES, m_triangleIDX.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
  }

  if(!m_quadIDX.empty())
  {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadIBO);
    glDrawElements(GL_QUADS, m_quadIDX.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
  }
#ifdef DEBUG
  tgCheckError("[tgRenderModel::DrawBufferObject] drawing quads");
#endif

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_INDEX_ARRAY);
  if(normals)
    glDisableClientState(GL_NORMAL_ARRAY);
  if(texture)
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

void tgRenderModel::Draw()
{
  this->DrawFaces();

  m_pose.Activate();
  this->DrawLines();
  this->DrawPoints();
  this->DrawColorPoints();
  m_pose.Deactivate();
}

void tgRenderModel::DrawFaces(bool lighting, RenderMode rmode)
{

  if (lighting) {
    m_material.Activate();
  } else {
    glDisable(GL_LIGHTING);
    glColor3f(m_material.color.x, m_material.color.y, m_material.color.z);
  }

  m_pose.Activate();
  switch (rmode) {
  case DISPLAYLIST:
    if (m_displaylist_initialized)
      glCallList(m_displaylist);
    else
      GenDisplayList();
    break;
  case BUFFEROBJECT:
    if (!m_bufferobject_initialized)
      GenBufferObject();
    DrawBufferObject();
    break;
  case RENDERNORMAL:
  default:
    tgModel::DrawFaces();
    break;

  }
  m_pose.Deactivate();

  if (lighting) {
    m_material.Deactivate();
  }
}

void tgRenderModel::DrawNormals(float normal_length)
{
  m_pose.Activate();
  tgModel::DrawNormals(normal_length);
  m_pose.Deactivate();
}
