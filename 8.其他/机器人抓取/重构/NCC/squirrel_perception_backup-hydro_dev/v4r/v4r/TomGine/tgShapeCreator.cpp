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

#include "tgShapeCreator.h"
#include "tgCollission.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdexcept>

using namespace TomGine;

// *************************************************************************************************
// PRIVATE
void tgShapeCreator::init_tetrahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices,
    unsigned &n_faces, unsigned &n_edges)
{
  float sqrt3 = 1.0f / sqrt(3.0f);
  float tetrahedron_vertices[] = { sqrt3, sqrt3, sqrt3, -sqrt3, -sqrt3, sqrt3, -sqrt3, sqrt3, -sqrt3, sqrt3, -sqrt3, -sqrt3 };
  int tetrahedron_faces[] = { 0, 2, 1, 0, 1, 3, 2, 3, 1, 3, 2, 0 };

  n_vertices = 4;
  n_faces = 4;
  n_edges = 6;
  vertices.clear();
  faces.clear();
  for (unsigned i = 0; i < (3 * n_vertices); i++)
    vertices.push_back(tetrahedron_vertices[i]);
  for (unsigned i = 0; i < (3 * n_faces); i++)
    faces.push_back(tetrahedron_faces[i]);
}

void tgShapeCreator::init_octahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices,
    unsigned &n_faces, unsigned &n_edges)
{
  float octahedron_vertices[] = { 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  int octahedron_faces[] = { 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 5, 2, 1, 5, 3, 2, 5, 4, 3, 5, 1, 4 };

  n_vertices = 6;
  n_faces = 8;
  n_edges = 12;
  vertices.clear();
  faces.clear();
  for (unsigned i = 0; i < (3 * n_vertices); i++)
    vertices.push_back(octahedron_vertices[i]);
  for (unsigned i = 0; i < (3 * n_faces); i++)
    faces.push_back(octahedron_faces[i]);
}

void tgShapeCreator::init_icosahedron(std::vector<float> &vertices, std::vector<int> &faces, unsigned &n_vertices,
    unsigned &n_faces, unsigned &n_edges)
{
  float t = (1 + sqrt(5.0f)) / 2;
  float tau = t / sqrt(1 + t * t);
  float one = 1 / sqrt(1 + t * t);

  float icosahedron_vertices[] = { tau, one, 0.0, -tau, one, 0.0, -tau, -one, 0.0, tau, -one, 0.0, one, 0.0, tau, one, 0.0, -tau,
      -one, 0.0, -tau, -one, 0.0, tau, 0.0, tau, one, 0.0, -tau, one, 0.0, -tau, -one, 0.0, tau, -one };
  int icosahedron_faces[] = { 4, 8, 7, 4, 7, 9, 5, 6, 11, 5, 10, 6, 0, 4, 3, 0, 3, 5, 2, 7, 1, 2, 1, 6, 8, 0, 11, 8, 11, 1, 9,
      10, 3, 9, 2, 10, 8, 4, 0, 11, 0, 5, 4, 9, 3, 5, 3, 10, 7, 8, 1, 6, 1, 11, 7, 2, 9, 6, 10, 2 };

  n_vertices = 12;
  n_faces = 20;
  n_edges = 30;
  vertices.clear();
  faces.clear();
  for (unsigned i = 0; i < (3 * n_vertices); i++)
    vertices.push_back(icosahedron_vertices[i]);
  for (unsigned i = 0; i < (3 * n_faces); i++)
    faces.push_back(icosahedron_faces[i]);

}

int tgShapeCreator::search_midpoint(int &index_start, int &index_end, unsigned &n_vertices, int &edge_walk,
    std::vector<int> &midpoint, std::vector<int> &start, std::vector<int> &end, std::vector<float> &vertices)
{
  int i;
  for (i = 0; i < edge_walk; i++)
    if ((start[i] == index_start && end[i] == index_end) || (start[i] == index_end && end[i] == index_start)) {
      int res = midpoint[i];

      /* update the arrays */
      start[i] = start[edge_walk - 1];
      end[i] = end[edge_walk - 1];
      midpoint[i] = midpoint[edge_walk - 1];
      edge_walk--;

      return res;
    }

  /* vertex not in the list, so we add it */
  start[edge_walk] = index_start;
  end[edge_walk] = index_end;
  midpoint[edge_walk] = n_vertices;

  /* create new vertex */
  vertices[3 * n_vertices] = (vertices[3 * index_start] + vertices[3 * index_end]) / 2.0f;
  vertices[3 * n_vertices + 1] = (vertices[3 * index_start + 1] + vertices[3 * index_end + 1]) / 2.0f;
  vertices[3 * n_vertices + 2] = (vertices[3 * index_start + 2] + vertices[3 * index_end + 2]) / 2.0f;

  /* normalize the new vertex */
  float length = sqrt(
      vertices[3 * n_vertices] * vertices[3 * n_vertices] + vertices[3 * n_vertices + 1] * vertices[3 * n_vertices + 1]
          + vertices[3 * n_vertices + 2] * vertices[3 * n_vertices + 2]);
  length = 1 / length;
  vertices[3 * n_vertices] *= length;
  vertices[3 * n_vertices + 1] *= length;
  vertices[3 * n_vertices + 2] *= length;

  n_vertices++;
  edge_walk++;
  return midpoint[edge_walk - 1];
}

void tgShapeCreator::subdivide(unsigned &n_vertices, unsigned &n_edges, unsigned &n_faces, std::vector<float> &vertices,
    std::vector<int> &faces)
{
  int n_vertices_new = n_vertices + 2 * n_edges;
  int n_faces_new = 4 * n_faces;
  unsigned i;

  int edge_walk = 0;
  n_edges = 2 * n_vertices + 3 * n_faces;

  std::vector<int> start(n_edges);
  std::vector<int> end(n_edges);
  std::vector<int> midpoint(n_edges);

  std::vector<int> faces_old = faces;
  vertices.resize(3 * n_vertices_new);
  faces.resize(3 * n_faces_new);
  n_faces_new = 0;

  for (i = 0; i < n_faces; i++) {
    int a = faces_old[3 * i];
    int b = faces_old[3 * i + 1];
    int c = faces_old[3 * i + 2];

    int ab_midpoint = search_midpoint(b, a, n_vertices, edge_walk, midpoint, start, end, vertices);
    int bc_midpoint = search_midpoint(c, b, n_vertices, edge_walk, midpoint, start, end, vertices);
    int ca_midpoint = search_midpoint(a, c, n_vertices, edge_walk, midpoint, start, end, vertices);

    faces[3 * n_faces_new] = a;
    faces[3 * n_faces_new + 1] = ab_midpoint;
    faces[3 * n_faces_new + 2] = ca_midpoint;
    n_faces_new++;
    faces[3 * n_faces_new] = ca_midpoint;
    faces[3 * n_faces_new + 1] = ab_midpoint;
    faces[3 * n_faces_new + 2] = bc_midpoint;
    n_faces_new++;
    faces[3 * n_faces_new] = ca_midpoint;
    faces[3 * n_faces_new + 1] = bc_midpoint;
    faces[3 * n_faces_new + 2] = c;
    n_faces_new++;
    faces[3 * n_faces_new] = ab_midpoint;
    faces[3 * n_faces_new + 1] = b;
    faces[3 * n_faces_new + 2] = bc_midpoint;
    n_faces_new++;
  }
  n_faces = n_faces_new;
}

// *************************************************************************************************
// PUBLIC
void tgShapeCreator::CreateSphere(tgModel& model, float radius, unsigned subdevisions, int method)
{
  unsigned i;
  int vidx = model.m_vertices.size();
  tgVertex v;
  tgFace f;

  std::vector<float> vertices;
  std::vector<int> faces;
  unsigned n_vertices;
  unsigned n_faces;
  unsigned n_edges;

  // Basic geometry used for sphere
  switch (method) {
  case TETRAHEDRON:
    init_tetrahedron(vertices, faces, n_vertices, n_faces, n_edges);
    break;
  case OCTAHEDRON:
    init_octahedron(vertices, faces, n_vertices, n_faces, n_edges);
    break;
  case ICOSAHEDRON:
    init_icosahedron(vertices, faces, n_vertices, n_faces, n_edges);
    break;
  default:
    init_icosahedron(vertices, faces, n_vertices, n_faces, n_edges);
    break;
  }

  // Subdevide basic geometry
  for (i = 0; i < subdevisions; i++)
    subdivide(n_vertices, n_edges, n_faces, vertices, faces);

  // Copy vertices
  for (i = 0; i < n_vertices; i++) {
    v.pos.x = radius * vertices[3 * i + 0];
    v.pos.y = radius * vertices[3 * i + 1];
    v.pos.z = radius * vertices[3 * i + 2];
    v.normal = v.pos;
    v.normal.normalize();
    model.m_vertices.push_back(v);
  }

  // Copy faces
  for (i = 0; i < n_faces; i++) {
    f.v.clear();
    f.v.push_back(vidx + faces[3 * i + 0]);
    f.v.push_back(vidx + faces[3 * i + 1]);
    f.v.push_back(vidx + faces[3 * i + 2]);
    model.m_faces.push_back(f);
  }

  //  if(vertices) free (vertices);
  //  if(faces) free (faces);

}

void tgShapeCreator::CreateBox(tgModel& model, float x, float y, float z)
{
  tgVertex v;
  tgFace f;
  int vidx = model.m_vertices.size();
  x = x * 0.5f;
  y = y * 0.5f;
  z = z * 0.5f;

  // Front
  v.pos = vec3(-x, -y, z);
  v.normal = vec3(0.0f, 0.0f, 1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, -y, z);
  v.normal = vec3(0.0f, 0.0f, 1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, z);
  v.normal = vec3(0.0f, 0.0f, 1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, y, z);
  v.normal = vec3(0.0f, 0.0f, 1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();

  // Back
  v.pos = vec3(x, -y, -z);
  v.normal = vec3(0.0f, 0.0f, -1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, -y, -z);
  v.normal = vec3(0.0f, 0.0f, -1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, y, -z);
  v.normal = vec3(0.0f, 0.0f, -1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, -z);
  v.normal = vec3(0.0f, 0.0f, -1.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();

  // Right
  v.pos = vec3(x, -y, z);
  v.normal = vec3(1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, -y, -z);
  v.normal = vec3(1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, -z);
  v.normal = vec3(1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, z);
  v.normal = vec3(1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();

  // Left
  v.pos = vec3(-x, -y, -z);
  v.normal = vec3(-1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, -y, z);
  v.normal = vec3(-1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, y, z);
  v.normal = vec3(-1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, y, -z);
  v.normal = vec3(-1.0f, 0.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();

  // Top
  v.pos = vec3(-x, y, z);
  v.normal = vec3(0.0f, 1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, z);
  v.normal = vec3(0.0f, 1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, y, -z);
  v.normal = vec3(0.0f, 1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, y, -z);
  v.normal = vec3(0.0f, 1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();

  // Bottom
  v.pos = vec3(x, -y, z);
  v.normal = vec3(0.0f, -1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, -y, z);
  v.normal = vec3(0.0f, -1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(-x, -y, -z);
  v.normal = vec3(0.0f, -1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  v.pos = vec3(x, -y, -z);
  v.normal = vec3(0.0f, -1.0f, 0.0f);
  model.m_vertices.push_back(v);
  f.v.push_back(vidx++);
  model.m_faces.push_back(f);
  f.v.clear();
}

void tgShapeCreator::CreateCylinder(tgModel &model, float radius, float height, unsigned slices, unsigned stacks, bool closed)
{
  unsigned x, z;
  int vidx = model.m_vertices.size();
  float alpha = 2 * M_PI / slices;
  float deltaH = height / stacks;
  tgVertex v[4];
  tgFace f;

  for (z = 0; z < stacks; z++) {
    for (x = 0; x < slices; x++) {
      f.v.clear();

      v[0].pos.x = radius * cos(alpha * x);
      v[0].pos.y = radius * sin(alpha * x);
      v[0].pos.z = deltaH * z - height * 0.5f;
      v[0].normal.x = v[0].pos.x;
      v[0].normal.y = v[0].pos.y;
      v[0].normal.z = 0.0f;
      v[0].normal = normalize(v[0].normal);
      model.m_vertices.push_back(v[0]);
      f.v.push_back(vidx++);

      v[1].pos.x = radius * cos(alpha * (x + 1));
      v[1].pos.y = radius * sin(alpha * (x + 1));
      v[1].pos.z = deltaH * z - height * 0.5f;
      v[1].normal.x = v[1].pos.x;
      v[1].normal.y = v[1].pos.y;
      v[1].normal.z = 0.0f;
      v[1].normal = normalize(v[1].normal);
      model.m_vertices.push_back(v[1]);
      f.v.push_back(vidx++);

      v[2].pos.x = radius * cos(alpha * (x + 1));
      v[2].pos.y = radius * sin(alpha * (x + 1));
      v[2].pos.z = deltaH * (z + 1) - height * 0.5f;
      v[2].normal.x = v[2].pos.x;
      v[2].normal.y = v[2].pos.y;
      v[2].normal.z = 0.0f;
      v[2].normal = normalize(v[2].normal);
      model.m_vertices.push_back(v[2]);
      f.v.push_back(vidx++);

      v[3].pos.x = radius * cos(alpha * x);
      v[3].pos.y = radius * sin(alpha * x);
      v[3].pos.z = deltaH * (z + 1) - height * 0.5f;
      v[3].normal.x = v[3].pos.x;
      v[3].normal.y = v[3].pos.y;
      v[3].normal.z = 0.0f;
      v[3].normal = normalize(v[3].normal);
      model.m_vertices.push_back(v[3]);
      f.v.push_back(vidx++);
      model.m_faces.push_back(f);
    }
  }

  if (closed) {
    for (z = 0; z < 2; z++) {
      deltaH = height * z - height * 0.5f;

      for (x = 0; x < slices; x++) {
        f.v.clear();

        v[0].pos = vec3(0.0, 0.0, deltaH);
        v[0].normal = normalize(v[0].pos);
        model.m_vertices.push_back(v[0]);
        f.v.push_back(vidx++);

        v[1].pos.x = radius * cos(alpha * x);
        v[1].pos.y = radius * sin(alpha * x);
        v[1].pos.z = deltaH;
        v[1].normal = v[0].normal;
        model.m_vertices.push_back(v[1]);
        f.v.push_back(vidx++);

        v[2].pos.x = radius * cos(alpha * (x + (float(z) - 0.5f) * 2.0f));
        v[2].pos.y = radius * sin(alpha * (x + (float(z) - 0.5f) * 2.0f));
        v[2].pos.z = deltaH;
        v[2].normal = v[0].normal;
        model.m_vertices.push_back(v[2]);
        f.v.push_back(vidx++);
        model.m_faces.push_back(f);
      }
    }
  }
}

void tgShapeCreator::CreateCone(tgModel &model, float radius, float height, unsigned slices, unsigned stacks, bool closed)
{

  if (stacks <= 0 || slices <= 2)
    return;

  unsigned x, z;
  int vidx = model.m_vertices.size();
  float alpha = 2 * M_PI / slices;
  float deltaH = height / stacks;
  float deltaR = radius / stacks;
  tgVertex v[4];
  tgFace f;

  for (z = 0; z < stacks; z++) {
    for (x = 0; x < slices; x++) {
      f.v.clear();

      v[0].pos.x = deltaR * z * cos(alpha * x);
      v[0].pos.y = deltaR * z * sin(alpha * x);
      v[0].pos.z = deltaH * z;
      v[0].normal.x = v[0].pos.x;
      v[0].normal.y = v[0].pos.y;
      v[0].normal.z = 0.0f;
      v[0].normal = normalize(v[0].normal);
      model.m_vertices.push_back(v[0]);
      f.v.push_back(vidx++);

      v[1].pos.x = deltaR * z * cos(alpha * (x + 1));
      v[1].pos.y = deltaR * z * sin(alpha * (x + 1));
      v[1].pos.z = deltaH * z;
      v[1].normal.x = v[1].pos.x;
      v[1].normal.y = v[1].pos.y;
      v[1].normal.z = 0.0f;
      v[1].normal = normalize(v[1].normal);
      model.m_vertices.push_back(v[1]);
      f.v.push_back(vidx++);

      v[2].pos.x = deltaR * (z + 1) * cos(alpha * (x + 1));
      v[2].pos.y = deltaR * (z + 1) * sin(alpha * (x + 1));
      v[2].pos.z = deltaH * (z + 1);
      v[2].normal.x = v[2].pos.x;
      v[2].normal.y = v[2].pos.y;
      v[2].normal.z = 0.0f;
      v[2].normal = normalize(v[2].normal);
      model.m_vertices.push_back(v[2]);
      f.v.push_back(vidx++);

      v[3].pos.x = deltaR * (z + 1) * cos(alpha * x);
      v[3].pos.y = deltaR * (z + 1) * sin(alpha * x);
      v[3].pos.z = deltaH * (z + 1);
      v[3].normal.x = v[3].pos.x;
      v[3].normal.y = v[3].pos.y;
      v[3].normal.z = 0.0f;
      v[3].normal = normalize(v[3].normal);
      model.m_vertices.push_back(v[3]);
      f.v.push_back(vidx++);
      model.m_faces.push_back(f);
    }
  }

  if (closed) {
    for (x = 0; x < slices; x++) {
      f.v.clear();

      v[0].pos = vec3(0.0f, 0.0f, height);
      v[0].normal = normalize(v[0].pos);
      model.m_vertices.push_back(v[0]);
      f.v.push_back(vidx++);

      v[1].pos.x = radius * cos(alpha * x);
      v[1].pos.y = radius * sin(alpha * x);
      v[1].pos.z = height;
      v[1].normal = v[0].normal;
      model.m_vertices.push_back(v[1]);
      f.v.push_back(vidx++);

      v[2].pos.x = radius * cos(alpha * (x + (float(z) - 0.5f) * 2.0f));
      v[2].pos.y = radius * sin(alpha * (x + (float(z) - 0.5f) * 2.0f));
      v[2].pos.z = height;
      v[2].normal = v[0].normal;
      model.m_vertices.push_back(v[2]);
      f.v.push_back(vidx++);
      model.m_faces.push_back(f);
    }
  }
}

void tgShapeCreator::CreatePlaneIndices(tgModel & model, unsigned vidx, unsigned segX, unsigned segY)
{
  tgFace f;

  for (unsigned j = 0; j < segY; j++) {
    for (unsigned i = 0; i < segX; i++) {

      int i0 = vidx + (segX + 1) * j + i;
      int i1 = vidx + (segX + 1) * j + i + 1;
      int i2 = vidx + (segX + 1) * (j + 1) + i + 1;
      int i3 = vidx + (segX + 1) * (j + 1) + i;

      f.v.push_back(i0);
      f.v.push_back(i1);
      f.v.push_back(i2);
      f.v.push_back(i3);

      model.m_faces.push_back(f);
      f.v.clear();

    }
  }
}

float tgShapeCreator::Area(const std::vector<vec2> &contour)
{

  int n = contour.size();

  float A = 0.0f;

  for (int p = n - 1, q = 0; q < n; p = q++) {
    A += contour[p].x * contour[q].y - contour[p].y * contour[q].x;
  }
  return A * 0.5f;
}

/*
 InsideTriangle decides if a point P is Inside of the triangle
 defined by A, B, C.
 */
bool tgShapeCreator::InsideTriangle(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float Px, float Py)

{
  float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
  float cCROSSap, bCROSScp, aCROSSbp;

  ax = Cx - Bx;
  ay = Cy - By;
  bx = Ax - Cx;
  by = Ay - Cy;
  cx = Bx - Ax;
  cy = By - Ay;
  apx = Px - Ax;
  apy = Py - Ay;
  bpx = Px - Bx;
  bpy = Py - By;
  cpx = Px - Cx;
  cpy = Py - Cy;

  aCROSSbp = ax * bpy - ay * bpx;
  cCROSSap = cx * apy - cy * apx;
  bCROSScp = bx * cpy - by * cpx;

  return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}
;

bool tgShapeCreator::Snip(const std::vector<vec2> &contour, int u, int v, int w, int n, int *V)
{
  int p;
  float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

  Ax = contour[V[u]].x;
  Ay = contour[V[u]].y;

  Bx = contour[V[v]].x;
  By = contour[V[v]].y;

  Cx = contour[V[w]].x;
  Cy = contour[V[w]].y;

  if (epsilon > (((Bx - Ax) * (Cy - Ay)) - ((By - Ay) * (Cx - Ax))))
    return false;

  for (p = 0; p < n; p++) {
    if ((p == u) || (p == v) || (p == w))
      continue;
    Px = contour[V[p]].x;
    Py = contour[V[p]].y;
    if (InsideTriangle(Ax, Ay, Bx, By, Cx, Cy, Px, Py))
      return false;
  }

  return true;
}

bool tgShapeCreator::TriangulatePolygon(const std::vector<vec2> &contour, std::vector<vec2> &result)
{
  /* allocate and initialize list of Vertices in polygon */

  int n = contour.size();
  if (n < 3)
    return false;

  int *V = new int[n];

  /* we want a counter-clockwise polygon in V */
  if (0.0f < Area(contour))
    for (int v = 0; v < n; v++)
      V[v] = v;
  else
    for (int v = 0; v < n; v++)
      V[v] = (n - 1) - v;

  int nv = n;

  /*  remove nv-2 Vertices, creating 1 triangle every time */
  int count = 2 * nv; /* error detection */

  for (int m = 0, v = nv - 1; nv > 2;) {
    /* if we loop, it is probably a non-simple polygon */
    if (0 >= (count--)) {
      //** Triangulate: ERROR - probable bad polygon!
      return false;
    }

    /* three consecutive vertices in current polygon, <u,v,w> */
    int u = v;
    if (nv <= u)
      u = 0; /* previous */
    v = u + 1;
    if (nv <= v)
      v = 0; /* new v    */
    int w = v + 1;
    if (nv <= w)
      w = 0; /* next     */

    if (Snip(contour, u, v, w, nv, V)) {
      int a, b, c, s, t;

      /* true names of the vertices */
      a = V[u];
      b = V[v];
      c = V[w];

      /* output Triangle */
      result.push_back(contour[a]);
      result.push_back(contour[b]);
      result.push_back(contour[c]);

      m++;

      /* remove v from remaining polygon */
      for (s = v, t = v + 1; t < nv; s++, t++)
        V[s] = V[t];
      nv--;

      /* resest error detection counter */
      count = 2 * nv;
    }
  }

  delete V;

  return true;
}

void tgShapeCreator::CreatePlaneXY(tgModel &model, float x0, float y0, float z0, float width, float height, unsigned segX,
    unsigned segY)
{
  tgVertex v;
  float dx = width / segX;
  float dy = height / segY;

  unsigned vidx = model.m_vertices.size();

  for (unsigned j = 0; j <= segY; j++) {
    for (unsigned i = 0; i <= segX; i++) {
      float x = x0 + i * dx;
      float y = y0 + j * dy;
      v.pos = vec3(x, y, z0);
      v.normal = vec3(0.0f, 0.0f, 1.0f);
      v.texCoord = vec2((x - x0) / width, (y - y0) / height);
      model.m_vertices.push_back(v);
    }
  }
  CreatePlaneIndices(model, vidx, segX, segY);
}

void tgShapeCreator::CreatePlaneYZ(tgModel &model, float x0, float y0, float z0, float width, float height, unsigned segY,
    unsigned segZ)
{
  tgVertex v;
  float dy = width / segY;
  float dz = height / segZ;

  unsigned vidx = model.m_vertices.size();

  for (unsigned j = 0; j <= segZ; j++) {
    for (unsigned i = 0; i <= segY; i++) {
      float y = y0 + i * dy;
      float z = z0 + j * dz;
      v.pos = vec3(x0, y, z);
      v.normal = vec3(1.0f, 0.0f, 0.0f);
      v.texCoord = vec2((y - y0) / width, (z - z0) / height);
      model.m_vertices.push_back(v);
    }
  }
  CreatePlaneIndices(model, vidx, segY, segZ);
}

void tgShapeCreator::CreatePlaneZX(tgModel &model, float x0, float y0, float z0, float width, float height, unsigned segZ,
    unsigned segX)
{
  tgVertex v;
  float dz = width / segZ;
  float dx = height / segX;

  unsigned vidx = model.m_vertices.size();

  for (unsigned j = 0; j <= segX; j++) {
    for (unsigned i = 0; i <= segZ; i++) {
      float z = z0 + i * dz;
      float x = x0 + j * dx;
      v.pos = vec3(x, y0, z);
      v.normal = vec3(0.0f, 1.0f, 0.0f);
      v.texCoord = vec2((z - z0) / width, (x - x0) / height);
      model.m_vertices.push_back(v);
    }
  }
  CreatePlaneIndices(model, vidx, segZ, segX);
}

void tgShapeCreator::CreateLine(tgModel &model, tgLine &line, unsigned seg)
{
  tgLine l;
  vec3 dir = (line.end - line.start) / seg;

  for (unsigned i = 0; i < seg; i++) {

    l.start = line.start + dir * i;
    l.end = line.start + dir* (i + 1);

    model.m_lines.push_back(l);
  }
}

void tgShapeCreator::PolygonMesh(tgModel& model, const std::vector<vec3> &points)
{
  if (points.size() < 3) {
    printf("[tgShapeCreator::PolygonMesh] Warning to few points!\n");
    return;
  }

  std::vector<vec2> a;

  for (unsigned i = 0; i < points.size(); i++) {
    a.push_back(vec2(points[i].x, points[i].y));
  }

  // allocate an STL vector to hold the answer.
  std::vector<vec2> result;

  //  Invoke the triangulator to triangulate this polygon.
  TriangulatePolygon(a, result);

  int tcount = result.size() / 3;
  unsigned idx(0);

  for (int i = 0; i < tcount; i++) {
    TomGine::tgFace f;

    for (int j = 0; j < 3; j++) {
      const vec2 &p = result[i * 3 + j];
      tgVertex vert;
      vert.pos = vec3(p.x, p.y, 0.0f);
      model.m_vertices.push_back(vert);
      f.v.push_back(idx++);
    }

    model.m_faces.push_back(f);
  }

  //  int i, s;
  //  int idx = model.m_vertices.size();
  //  bool pointInTriangle;
  //  bool pointIsConvex;
  //  vec3 e1, e2, n, poly_normal;
  //  vec3 v0, v1, v2;
  //  tgVertex v;
  //  tgFace f;
  //  std::vector<tgVertex> vertices;
  //  std::vector<tgVertex>::iterator v_act, v_pre, v_post, v_in;
  //
  //  s = points.size();
  //  if( s < 3 ) {
  //    printf("[tgModel::TriangulatePolygon] Warning to few points for polygon: %d\n", s);
  //    return;
  //  }
  //
  //  for( i = 0; i < s; i++ ) {
  //    // Calculate normal
  //    v0 = points[i];
  //    v1 = points[(i + 1) % s];
  //    v2 = points[(i + s - 1) % s];
  //    e1 = v1 - v0;
  //    e2 = v2 - v0;
  //    e1.normalize();
  //    e2.normalize();
  //    n.cross(e1, e2);
  //    poly_normal = poly_normal + n; // polygon normal = sum of all normals
  //
  //    // check if same point is already in list
  //    bool valid(true);
  //    for( unsigned j = 0; j < vertices.size(); j++ ) {
  //      vec3 vc = vertices[j].pos;
  //      if( v0.x == vc.x && v0.y == vc.y && v0.z == vc.z ) {
  //        //        throw std::runtime_error("tgShapeCreator::TriangulatePolygon: double entry");
  //        printf("[tgShapeCreator::TriangulatePolygon] Warning double entry in point list\n");
  //        valid = false;
  //      }
  //    }
  //
  //    if( valid ) {
  //      v.pos = v0;
  //      vertices.push_back(v);
  //    }
  //
  //    if( v0.x == 0.0f && v0.y == 0.0f && v0.z == 0.0f )
  //      printf("[tgShapeCreator::TriangulatePolygon] Warning zero point\n");
  //
  //  }
  //
  //  poly_normal.normalize(); // normalize polygon normal
  //  if(poly_normal.z > 0.0)
  //    poly_normal = -poly_normal;
  //
  //  v_act = vertices.begin();
  //  while( vertices.size() > 2 ) {
  //
  //
  //    if( v_act >= vertices.end() ) {
  //      v_act = vertices.end() - 1;
  //      v_pre = v_act - 1;
  //      v_post = vertices.begin();
  //    } else if( v_act == vertices.begin() ) {
  //      v_act = vertices.begin();
  //      v_pre = vertices.end() - 1;
  //      v_post = v_act + 1;
  //    } else {
  //      v_pre = v_act - 1;
  //      v_post = v_act + 1;
  //    }
  ////    printf("vertices: \n");
  ////    for( unsigned j = 0; j < vertices.size(); j++ ) {
  ////      printf("  v: %f %f %f\n", vertices[j].pos.x, vertices[j].pos.y, vertices[j].pos.z);
  ////    }
  ////    printf("   v_pre: %f %f %f, ", (*v_pre).pos.x, (*v_pre).pos.y, (*v_pre).pos.z);
  ////    printf("   v_act: %f %f %f, ", (*v_act).pos.x, (*v_act).pos.y, (*v_act).pos.z);
  ////    printf("   v_post: %f %f %f, ", (*v_post).pos.x, (*v_post).pos.y, (*v_post).pos.z);
  ////    printf("\n");
  //
  //    // Test if triangle is convex
  //    v0 = (*v_act).pos;
  //    v1 = (*v_post).pos;
  //    v2 = (*v_pre).pos;
  //    e1 = v1 - v0;
  //    e2 = v2 - v0;
  //    e1.normalize();
  //    e2.normalize();
  //    (*v_act).normal.cross(e1, e2);
  //    (*v_act).normal.normalize();
  //
  //    if( (*v_act).normal * poly_normal >= 0.0 ) {
  //      pointIsConvex = true;
  //    }
  //
  //    // Test if any other point of remaining polygon lies in this triangle
  //    pointInTriangle = false;
  //    if( pointIsConvex ) {
  //      pointInTriangle = false;
  //      for( v_in = vertices.begin(); v_in < vertices.end() && vertices.size() > 3; v_in++ ) {
  //        if( v_in != v_act && v_in != v_pre && v_in != v_post )
  //          pointInTriangle = tgCollission::PointInTriangle((*v_in).pos, (*v_post).pos, (*v_act).pos, (*v_pre).pos);
  //      }
  //    }
  //
  //    if( pointIsConvex && !pointInTriangle ) {
  //      // Generate face
  //      f.v.clear();
  //
  //      (*v_pre).normal = poly_normal;
  //      (*v_act).normal = poly_normal;
  //      (*v_post).normal = poly_normal;
  //
  //      model.m_vertices.push_back(*v_pre);
  //      f.v.push_back(idx);
  //      idx++;
  //      model.m_vertices.push_back(*v_act);
  //      f.v.push_back(idx);
  //      idx++;
  //      model.m_vertices.push_back(*v_post);
  //      f.v.push_back(idx);
  //      idx++;
  //
  //      f.normal = poly_normal;
  //      model.m_faces.push_back(f);
  //      v_act = vertices.erase(v_act);
  //
  //      //      printf("face: %d, %d %d %d, %d, %d\n", model.m_faces.size() - 1, f.v[0], f.v[1], f.v[2], model.m_vertices.size(), vertices.size());
  //      //      printf("   %d: %f %f %f\n", f.v[0], model.m_vertices[f.v[0]].pos.x, model.m_vertices[f.v[0]].pos.y, model.m_vertices[f.v[0]].pos.z);
  //      //      printf("   %d: %f %f %f\n", f.v[1], model.m_vertices[f.v[1]].pos.x, model.m_vertices[f.v[1]].pos.y, model.m_vertices[f.v[1]].pos.z);
  //      //      printf("   %d: %f %f %f\n", f.v[2], model.m_vertices[f.v[2]].pos.x, model.m_vertices[f.v[2]].pos.y, model.m_vertices[f.v[2]].pos.z);
  //
  //    } else {
  //      v_act = v_post;
  //    }
  //
  //  }
  //  vertices.clear();
}

