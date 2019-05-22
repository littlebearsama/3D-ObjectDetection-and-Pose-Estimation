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

#include "tgModel.h"
#include "headers.h"
#include "tgShapeCreator.h"

#include <map>

using namespace TomGine;

//tgModel& tgModel::operator+=(const tgModel& m)
void tgModel::Merge(const tgModel &m)
{
  unsigned vi = m_vertices.size();
  for (unsigned i = 0; i < m.m_vertices.size(); i++)
    m_vertices.push_back(m.m_vertices[i]);

  for (unsigned i = 0; i < m.m_faces.size(); i++)
  {
    m_faces.push_back(m.m_faces[i]);
    TomGine::tgFace &f = m_faces[m_faces.size() - 1];
    for (unsigned j = 0; j < f.v.size(); j++)
      f.v[j] += vi;
  }

  for (unsigned i = 0; i < m.m_lines.size(); i++)
    m_lines.push_back(m.m_lines[i]);

  for (unsigned i = 0; i < m.m_points.size(); i++)
    m_points.push_back(m.m_points[i]);

  for (unsigned i = 0; i < m.m_colorpoints.size(); i++)
    m_colorpoints.push_back(m.m_colorpoints[i]);

}

void tgModel::Draw()
{
  DrawFaces();
  DrawLines();
  DrawPoints();
  DrawColorPoints();
}

void tgModel::DrawVertices() const
{
  glBegin(GL_POINTS);
  for (unsigned i = 0; i < m_vertices.size(); i++)
  {
    const tgVertex& v = m_vertices[i];
    glTexCoord2f(v.texCoord.x, v.texCoord.y);
    glNormal3f(v.normal.x, v.normal.y, v.normal.z);
    glVertex3f(v.pos.x, v.pos.y, v.pos.z);
  }
  glEnd();
}

void tgModel::DrawFaces() const
{
  glLineWidth(m_line_width); // for wire-frame mode

  if(m_coloring==FULL_COLORING)
    glDisable(GL_COLOR_MATERIAL);
  else
    glEnable(GL_COLOR_MATERIAL);

  for (size_t i = 0; i < m_faces.size(); i++)
  {
    const tgFace &f = m_faces[i];

    if (f.v.size() == 3)
      glBegin(GL_TRIANGLES);
    else if (f.v.size() == 4)
      glBegin(GL_QUADS);
    else
    {
      printf("[tgModel::DrawFaces()] Warning, no suitable face format\n");
      printf("[tgModel::DrawFaces()] Face has %d vertices (supported: 1,2,3 or 4)\n", (int) f.v.size());
      glEnd();
      continue;
    }

    if(m_coloring==PER_FACE_COLORING)
      glColor4ub(f.color[0],f.color[1],f.color[2],f.color[3]);

    for (size_t j = 0; j < f.v.size(); j++)
    {
      const tgVertex& v = m_vertices[f.v[j]];
      glTexCoord2f(v.texCoord.x, v.texCoord.y);
      glNormal3f(v.normal.x, v.normal.y, v.normal.z);
      if(m_coloring==PER_VERTEX_COLORING)
        glColor3ub(v.color[0], v.color[1], v.color[2]);
      glVertex3f(v.pos.x, v.pos.y, v.pos.z);
    }
    glEnd();
  }

  glDisable(GL_COLOR_MATERIAL);
}

void tgModel::DrawLines() const
{
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glColor3f(m_line_color.x, m_line_color.y, m_line_color.z);
  glLineWidth(m_line_width);

  glBegin(GL_LINES);
  for (unsigned i = 0; i < m_lines.size(); i++)
  {
    const tgLine& line = m_lines[i];
    glVertex3f(line.start.x, line.start.y, line.start.z);
    glVertex3f(line.end.x, line.end.y, line.end.z);
  }
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
}

void tgModel::DrawPoints() const
{
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glColor3f(m_point_color.x, m_point_color.y, m_point_color.z);
  glPointSize(m_point_size);

  glBegin(GL_POINTS);
  for (unsigned i = 0; i < m_points.size(); i++)
  {
    const vec3& p = m_points[i];
    glVertex3f(p.x, p.y, p.z);
  }
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
}

void tgModel::DrawColorPoints() const
{
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glPointSize(m_point_size);

  glBegin(GL_POINTS);
  for (unsigned i = 0; i < m_colorpoints.size(); i++)
  {
    const tgColorPoint& cp = m_colorpoints[i];
    glColor3ub(cp.color[0], cp.color[1], cp.color[2]);
    glVertex3f(cp.pos.x, cp.pos.y, cp.pos.z);
  }
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
}

void tgModel::DrawNormals(float length) const
{ // draw normals
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glLineWidth(m_line_width);

  glBegin(GL_LINES);

  for (unsigned j = 0; j < m_vertices.size(); j++)
  {
    const tgVertex& v = m_vertices[j];
    glVertex3f(v.pos.x, v.pos.y, v.pos.z);
    glVertex3f(v.pos.x + v.normal.x * length, v.pos.y + v.normal.y * length, v.pos.z + v.normal.z * length);
  }
  glEnd();

  glColor3f(1.0f, 1.0f, 1.0f);
}

bool tgModel::CheckFaces() const
{
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    const TomGine::tgFace &f = m_faces[i];

    if (f.v.size() != 3 && f.v.size() != 4)
    {
      printf("[tgModel::CheckFaces()] No suitable face format\n");
      printf("[tgModel::CheckFaces()] Face has %d vertices (supported: 3 or 4)\n", (int) m_faces[i].v.size());
      return false;
    }

    for (size_t j = 0; j < f.v.size(); j++)
    {
      if (f.v[j] < 0 || f.v[j] >= m_vertices.size())
      {
        printf("[tgModel::CheckFaces()] Vertex index out of bounds (0 <= %d < %lu)\n", f.v[j], m_vertices.size());
        return false;
      }
    }
  }
  return true;
}

void tgModel::RemoveUnusedVertices()
{
  // get vertices in use
  std::vector<bool> v_used(m_vertices.size(), false);
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    TomGine::tgFace &f = m_faces[i];
    for (size_t j = 0; j < f.v.size(); j++)
      v_used[f.v[j]] = true;
  }

  // calculate relation between old and new indices
  std::map<size_t, size_t> idxmap;
  size_t idx(0);
  for (size_t i = 0; i < v_used.size(); i++)
  {
    if (v_used[i] == true)
    {
      idxmap[i] = idx;
      idx++;
    }
  }

  // update indices of faces
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    TomGine::tgFace &f = m_faces[i];
    for (size_t j = 0; j < f.v.size(); j++)
      f.v[j] = idxmap[f.v[j]];
  }

  // reallocate vertices
  std::vector<TomGine::tgVertex> vertices(idxmap.size());
  std::map<size_t, size_t>::iterator it;
  for (it = idxmap.begin(); it != idxmap.end(); it++)
    vertices[it->second] = m_vertices[it->first];

  m_vertices = vertices;
}

void tgModel::RemoveDublicateVertices(bool quiet)
{
  if(m_vertices.size()<2)
    return;


  // find dublicates and store vertex index mapping
  std::map< size_t, size_t > idxmap;
  std::vector<bool> dublicate(m_vertices.size(), false);
  std::vector<tgVertex>::iterator first, curr, found;
  first = curr = m_vertices.begin();
  curr++;
  size_t idx(1);
  size_t rem(0);
  while(curr!=m_vertices.end())
  {
    found = std::find(first,curr,*curr);
    if(found==curr)
    {
      idxmap[idx] = idx - rem;
    }else
    {
      idxmap[idx] = idxmap[std::distance(first, found)];
      dublicate[idx] = true;
      rem++;
    }

    idx++;
    curr++;
  }

  // remove dublicates
  std::vector<tgVertex> vertices;
  for(size_t i=0; i<m_vertices.size(); i++)
    if(!dublicate[i])
      vertices.push_back(m_vertices[i]);
  m_vertices = vertices;

  // update indices of faces
  for(size_t i=0; i<m_faces.size(); i++)
    for(size_t j=0; j<m_faces[i].v.size(); j++)
    {
      unsigned& v = m_faces[i].v[j];
      v = idxmap[v];
    }

  if(!quiet)
    printf("[tgModel::RemoveDublicateVertices] removed: %lu\n", rem);
}

void tgModel::DublicateCommonVertices()
{
  std::vector<bool> vertex_used(m_vertices.size(), false);

  for (size_t i = 0; i < m_faces.size(); i++)
  {
    TomGine::tgFace &f = m_faces[i];

    for (size_t j = 0; j < f.v.size(); j++)
    {
      unsigned vidx = f.v[j];

      if (vertex_used[vidx])
      {
        f.v[j] = m_vertices.size();
        m_vertices.push_back(m_vertices[vidx]);
        vertex_used.push_back(true);
      } else
      {
        vertex_used[vidx] = true;
      }

    }
  }
}

//void tgModel::MergeDublicatedVertices(float threshold)
//{
//
//}

// Compute normal vectors of vertices
void tgModel::ComputeNormals()
{
  ComputeFaceNormals();

  std::map<size_t, std::vector<size_t> > vertex_faces;

  for (size_t i = 0; i < m_faces.size(); i++)
  {
    tgFace &f = m_faces[i];
    for (size_t j = 0; j < m_faces[i].v.size(); j++)
      vertex_faces[f.v[j]].push_back(i);
  }

  for (size_t i = 0; i < vertex_faces.size(); i++)
  {
    tgVertex &v = m_vertices[i];
    v.normal = vec3(0.0, 0.0, 0.0);
    for (size_t j = 0; j < vertex_faces[i].size(); j++)
      v.normal += m_faces[vertex_faces[i][j]].normal;

    v.normal.normalize();
  }

  //  unsigned i, j;
  //  tgFace* f;
  //  vec3 v0, v1, v2, e1, e2, n;
  //
  //  // calculate vertex normals using the face normal
  //  for (i = 0; i < m_faces.size(); i++) {
  //    f = &m_faces[i];
  //
  //    v0 = vec3(m_vertices[f->v[0]].pos.x, m_vertices[f->v[0]].pos.y, m_vertices[f->v[0]].pos.z);
  //    v1 = vec3(m_vertices[f->v[1]].pos.x, m_vertices[f->v[1]].pos.y, m_vertices[f->v[1]].pos.z);
  //    v2 = vec3(m_vertices[f->v[2]].pos.x, m_vertices[f->v[2]].pos.y, m_vertices[f->v[2]].pos.z);
  //    e1 = v1 - v0;
  //    e2 = v2 - v0;
  //
  //    n.cross(e1, e2);
  //    n.normalize();
  //    f->normal = vec3(n);
  //    for (j = 0; j < m_faces[i].v.size(); j++) {
  //      m_vertices[f->v[j]].normal.x = n.x;
  //      m_vertices[f->v[j]].normal.y = n.y;
  //      m_vertices[f->v[j]].normal.z = n.z;
  //    }
  //  }
}

void tgModel::FlipNormals()
{
  for (size_t i = 0; i < m_vertices.size(); i++)
    m_vertices[i].normal *= -1.0f;
}

void tgModel::ComputeFaceNormals()
{
  vec3 v0, v1, v2, e1, e2, n;

  // calculate vertex normals using the face normal
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    tgFace &f = m_faces[i];

    f.normal = vec3(0,0,0);

    if(f.v.size()<3)
      continue;

    for(size_t j=0; j<f.v.size()-2; j++)
    {
      v0 = vec3(m_vertices[f.v[j+0]].pos.x, m_vertices[f.v[j+0]].pos.y, m_vertices[f.v[j+0]].pos.z);
      v1 = vec3(m_vertices[f.v[j+1]].pos.x, m_vertices[f.v[j+1]].pos.y, m_vertices[f.v[j+1]].pos.z);
      v2 = vec3(m_vertices[f.v[j+2]].pos.x, m_vertices[f.v[j+2]].pos.y, m_vertices[f.v[j+2]].pos.z);
      e1 = v1 - v0;
      e2 = v2 - v0;
      n.cross(e1, e2);
      f.normal += n;
    }
    f.normal.normalize();
  }
}

void tgModel::FlipFaceNormals()
{
  for (size_t i = 0; i < m_faces.size(); i++)
    m_faces[i].normal *= -1.0f;
}

void tgModel::FlipFaces()
{
  for (size_t i = 0; i < m_faces.size(); i++)
  {
    tgFace &f = m_faces[i];

    std::vector<unsigned> v;
    for (size_t j = 0; j < f.v.size(); j++)
      v.push_back(f.v[f.v.size() - 1 - j]);

    f.v = v;
  }
}

void tgModel::ComputeBoundingSphere()
{
  vec3 _min;
  vec3 _max;

  ComputeBoundingSphere(_min, _max);
}

void tgModel::ComputeBoundingSphere(vec3 &_min, vec3 &_max)
{
  vec3 v;

  if (m_vertices.empty())
    return;

  _min = m_vertices[0].pos;
  _max = _min;

  for (unsigned i = 1; i < m_vertices.size(); i++)
  {
    v = m_vertices[i].pos;
    if (v.x < _min.x)
      _min.x = v.x;
    if (v.y < _min.y)
      _min.y = v.y;
    if (v.z < _min.z)
      _min.z = v.z;

    if (v.x > _max.x)
      _max.x = v.x;
    if (v.y > _max.y)
      _max.y = v.y;
    if (v.z > _max.z)
      _max.z = v.z;
  }
  m_bs.center.x = (_max.x + _min.x) * 0.5f;
  m_bs.center.y = (_max.y + _min.y) * 0.5f;
  m_bs.center.z = (_max.z + _min.z) * 0.5f;

  float rad = 0.0f;
  for (unsigned i = 0; i < m_vertices.size(); i++)
  {
    v = m_vertices[i].pos - m_bs.center;
    float rv = v.x * v.x + v.y * v.y + v.z * v.z;
    if (rad < rv)
      rad = rv;
  }

  m_bs.radius = sqrt(rad);

}

void tgModel::Clear()
{
  m_vertices.clear();
  m_faces.clear();
  m_lines.clear();
  m_points.clear();
  m_colorpoints.clear();
}

void tgModel::Print() const
{

  for (unsigned i = 0; i < m_vertices.size(); i++)
  {
    printf("Vertex %d: %f %f %f, %f %f %f\n", i, m_vertices[i].pos.x, m_vertices[i].pos.y, m_vertices[i].pos.z,
           m_vertices[i].normal.x, m_vertices[i].normal.y, m_vertices[i].normal.z);
  }

  for (unsigned i = 0; i < m_faces.size(); i++)
  {

    printf("Face %d:", i);
    for (unsigned j = 0; j < m_faces[i].v.size(); j++)
    {
      printf(" %d", m_faces[i].v[j]);
    }
    printf("\n");
  }
}

// void tgModel::ComputeQuadstripNormals(){
// 	int i,j,s;
// 	Face* f;
// 	vec3 v0, v1, v2, e1, e2, n, n1, n2;
// 	
// 	for(i=0; i<(int)m_quadstrips.size(); i++){
// 		f = &m_quadstrips[i];
// 		s = (int)f->v.size();
// 		for(j=0; j<(int)s; j++){
// 				
// 			v0 = vec3(m_vertices[f->v[(j+0)%s]].pos.x, m_vertices[f->v[(j+0)%s]].pos.y, m_vertices[f->v[(j+0)%s]].pos.z);
// 			v1 = vec3(m_vertices[f->v[(j+1)%s]].pos.x, m_vertices[f->v[(j+1)%s]].pos.y, m_vertices[f->v[(j+1)%s]].pos.z);
// 			v2 = vec3(m_vertices[f->v[(j+2)%s]].pos.x, m_vertices[f->v[(j+2)%s]].pos.y, m_vertices[f->v[(j+2)%s]].pos.z);
// 			e1 = v1 - v0;
// 			e2 = v2 - v0;
// 			n1.cross(e1,e2);
// 			n1.normalize();
// 			
// 			v0 = vec3(m_vertices[f->v[(j+0)%s]].pos.x, m_vertices[f->v[(j+0)%s]].pos.y, m_vertices[f->v[(j+0)%s]].pos.z);
// 			v1 = vec3(m_vertices[f->v[(j+s-1)%s]].pos.x, m_vertices[f->v[(j+s-1)%s]].pos.y, m_vertices[f->v[(j+s-1)%s]].pos.z);
// 			v2 = vec3(m_vertices[f->v[(j+s-2)%s]].pos.x, m_vertices[f->v[(j+s-2)%s]].pos.y, m_vertices[f->v[(j+s-2)%s]].pos.z);
// 			e1 = v1 - v0;
// 			e2 = v2 - v0;
// 			n2.cross(e2,e1);
// 			n2.normalize();
// 			
// 			n = (n1 + n2) * 0.5;
// 			
// 			if(j%2) n = n * -1.0;
// 			
// 			m_vertices[f->v[j]].normal.x = n.x;
// 			m_vertices[f->v[j]].normal.y = n.y;
// 			m_vertices[f->v[j]].normal.z = n.z;
// 		}
// 	}
// }


// void tgModel::DrawTriangleFan() const{
// 	int i,j,v;
// 	for(i=0; i<(int)m_trianglefans.size(); i++){
// 		glBegin(GL_TRIANGLE_FAN);		
// 			for(j=0; j<(int)m_trianglefans[i].v.size(); j++){
// 				v = m_trianglefans[i].v[j];
// 				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
// 				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
// 				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
// 			}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawQuadstrips() const{
// 	int i,j,v;
// 	for(i=0; i<(int)m_quadstrips.size(); i++){
// 		glBegin(GL_QUAD_STRIP);		
// 			for(j=0; j<(int)m_quadstrips[i].v.size(); j++){
// 				v = m_quadstrips[i].v[j];
// 				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
// 				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
// 				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
// 			}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawLines() const{
// 	for(int i=0; i<(int)m_lines.size(); i++){
// 		glBegin(GL_LINES);
// 			glVertex3f(m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z);
// 			glVertex3f(m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawLineLoops() const{
// 	int i,j;
// 	vec3 p;
// 	for(i=0; i<(int)m_lineloops.size(); i++){
// 		glBegin(GL_LINE_LOOP);
// 		for(j=0; j<(int)m_lineloops[i].points.size(); j++){
// 				p = m_lineloops[i].points[j];
// 				glVertex3f(p.x, p.y, p.z);
// 		}
// 		glEnd();
// 	}
// }
// 
// void tgModel::DrawPoints() const{
// 	glDisable(GL_LIGHTING);
// 	for(int i=0; i<(int)m_points.size(); i++){
// 		glBegin(GL_POINTS);
// 			glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
// 		glEnd();
// 	}
// 	glEnable(GL_LIGHTING);
// }


