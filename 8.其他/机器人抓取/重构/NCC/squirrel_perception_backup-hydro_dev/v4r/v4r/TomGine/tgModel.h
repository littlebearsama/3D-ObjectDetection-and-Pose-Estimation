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

#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>
#include <string>

#include "tgMathlib.h"

namespace TomGine{

/** @brief Vertex representing a directed point in 3D space with texture coordinates. See glVertex, glNormal, glTexCoord in OpenGL spec. */
struct tgVertex{
  vec3 pos;				///< 3D position of vertex
  vec3 normal;			///< Normal vector of vertex
  vec2 texCoord;			///< Texture coordinate of vertex
  unsigned char color[4];
  tgVertex()
  {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 255;
  }
  tgVertex(float x, float y, float z)
  {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 255;
    pos.x = x;
    pos.y = y;
    pos.z = z;
  }
  tgVertex(vec3 p)
  {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 255;
    pos = p;
  }
  bool operator==(const tgVertex &rhs)
  {
    return (this->pos==rhs.pos);
  }
};
/** @brief Face of a surface built by indexed vertices. See glBegin(GL_QUAD), glBegin(GL_TRIANGLE) in OpenGL spec. */
struct tgFace{
  std::vector<unsigned> v;///< List of vertex-indices
  vec3 normal;			///< Normal vector of face
  unsigned char color[4];
  tgFace()
  {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 255;
  }
  void Color(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
  {
    color[0] = r;
    color[1] = g;
    color[2] = b;
    color[3] = a;
  }
};
/** @brief Line defined by a starting and end point in 3D space. See glBegin(GL_LINE) in OpenGL spec. */
struct tgLine{
  tgLine(){};
  tgLine(vec3 s, vec3 e) : start(s), end(e){};
  vec3 start;	///< Start point of the line.
  vec3 end;	///< End point of line.
};
/** @brief Ray define by starting point and direction. */
struct tgRay{
  tgRay(){}
  tgRay(vec3 s, vec3 e) : start(s), dir(e){}
  vec3 start;	///< Start point of the ray.
  vec3 dir;	///< Vector/direction of the ray (view ray).
};
/** @brief A point in 3D with a color assigned. See glBegin(GL_POINTS), glColor in OpenGL spec. */
struct tgColorPoint{
  vec3 pos;					///< Position of the point.
  unsigned char color[3];		///< Color of the point in RGB [0..255].
};
/** @brief Bounding sphere of the model defined by the center and radius. */
struct BoundingSphere{
  vec3 center;	///< Center point of the sphere.
  float radius;	///< Radius of the sphere.
  BoundingSphere() : center(0.0f,0.0f,0.0f), radius(-1.0f){}
};
/** @brief Rectangle in 2D defined by a point (x,y), width and height. */
struct tgRect2D{
  float x,y,w,h;
  tgRect2D(float x, float y, float w, float h){ this->x=x; this->y=y; this->w=w; this->h=h;}
  tgRect2D(){ this->x=0.0f; this->y=0.0f; this->w=1.0f; this->h=1.0f;}
};

struct tgRect2Di{
  int x,y, w,h;
  tgRect2Di(int x, int y, int w, int h){ this->x=x; this->y=y; this->w=w; this->h=h;}
  tgRect2Di(){ this->x=0; this->y=0; this->w=2; this->h=2;}
};

/** @brief Geometric representation of various primitives (triangles, quadrangles, lines, points, ...) */
class tgModel
{
public:
  enum Coloring
  {
    FULL_COLORING=0,
    PER_FACE_COLORING=1,
    PER_VERTEX_COLORING=2
  } m_coloring;

  std::string name;
  std::vector<tgVertex>	m_vertices;				///< list of vertices
  std::vector<tgFace>		m_faces;				///< list of faces
  std::vector<tgLine>		m_lines;				///< list of lines
  std::vector<vec3>		m_points;				///< list of points
  std::vector<tgColorPoint> m_colorpoints;		///< list of colored points
  BoundingSphere  		m_bs;					///< bounding sphere

  float m_point_size;
  float m_line_width;
  vec3 m_line_color;
  vec3 m_point_color;

  tgModel() :
    m_coloring(FULL_COLORING),
    m_point_size(1.0f),
    m_line_width(1.0f),
    m_line_color(1.0, 0.0, 0.0),
    m_point_color(1.0, 0.0, 0.0)
  {}
  virtual ~tgModel() {}



  //	virtual tgModel& operator+=(const tgModel& m);
  virtual void Merge(const tgModel &m);

  /** @brief Save data access to vertices
   *  @param i	index of vertex in list m_vertices */
  tgVertex	getVertex(unsigned int i){ if(i<m_vertices.size() && i>=0) return m_vertices[i]; else return tgVertex();}

  /** @brief Save data access to faces
   *  @param i	index of face in list m_faces */
  tgFace		getFace(unsigned int i){ if(i<m_faces.size() && i>=0) return m_faces[i]; else return tgFace();}

  /** @brief Draw all data in model. */
  virtual void Draw();

  /** @brief Draw vertices as points. */
  virtual void DrawVertices() const;

  /** @brief Draws triangles and quadrangles given by m_faces. */
  virtual void DrawFaces() const;

  /** @brief Draws lines given by m_lines all with the color given. */
  virtual void DrawLines() const;

  /** @brief Draws points given by m_points all with the color given. */
  virtual void DrawPoints() const;

  /** @brief Draws colored points, given by m_colorpoints. */
  virtual void DrawColorPoints() const;

  /** @brief Draws normals of vertices in m_faces.
   *  @param length The length of the line representing the normal. */
  virtual void DrawNormals(float length = 1.0) const;

  /** @brief Checks for consistence of faces. */
  virtual bool CheckFaces() const;

  /** @brief Remove vertices that are not in use by any face */
  virtual void RemoveUnusedVertices();

  /** @brief Removes dublicate vertices (removes sharp edges) */
  virtual void RemoveDublicateVertices(bool quiet=true);

  /** @brief dublicate vertices that are in use by more than one face */
  virtual void DublicateCommonVertices ();

  /** @brief Compute normals of vertices using cross product of faces. */
  virtual void ComputeNormals();

  /** @brief Flip normals of vertices */
  virtual void FlipNormals();

  /** @brief Compute normals of vertices of m_faces, m_polygons, m_quadstrips. */
  virtual void ComputeFaceNormals();

  /** @brief Flip normals of vertices of m_faces, m_polygons, m_quadstrips. */
  virtual void FlipFaceNormals();

  /** @brief Flip faces. I.e. change direction of rotation (needed for backface culling) */
  virtual void FlipFaces ();

  /** @brief Compute bounding sphere which contains all vertices.*/
  virtual void ComputeBoundingSphere();
  virtual void ComputeBoundingSphere(vec3 &_min, vec3 &_max);

  /** @brief Clears data of model (m_vertices and m_faces). */
  virtual void Clear();

  /** @brief Prints infos of model to console. */
  virtual void Print() const;

};

} // namespace TomGine

#endif
