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

#include "tgFrustum.h"

using namespace TomGine;

void
tgFrustum::ExtractFrustum (const mat4 &i, const mat4 &e)
{
  float clip[16];
  float t;

  m_intrinsic = i;
  m_extrinsic = e;

  /* Combine the two matrices (multiply projection by modelview) */
  clip[0] = e[0] * i[0] + e[1] * i[4] + e[2] * i[8] + e[3] * i[12];
  clip[1] = e[0] * i[1] + e[1] * i[5] + e[2] * i[9] + e[3] * i[13];
  clip[2] = e[0] * i[2] + e[1] * i[6] + e[2] * i[10] + e[3] * i[14];
  clip[3] = e[0] * i[3] + e[1] * i[7] + e[2] * i[11] + e[3] * i[15];

  clip[4] = e[4] * i[0] + e[5] * i[4] + e[6] * i[8] + e[7] * i[12];
  clip[5] = e[4] * i[1] + e[5] * i[5] + e[6] * i[9] + e[7] * i[13];
  clip[6] = e[4] * i[2] + e[5] * i[6] + e[6] * i[10] + e[7] * i[14];
  clip[7] = e[4] * i[3] + e[5] * i[7] + e[6] * i[11] + e[7] * i[15];

  clip[8] = e[8] * i[0] + e[9] * i[4] + e[10] * i[8] + e[11] * i[12];
  clip[9] = e[8] * i[1] + e[9] * i[5] + e[10] * i[9] + e[11] * i[13];
  clip[10] = e[8] * i[2] + e[9] * i[6] + e[10] * i[10] + e[11] * i[14];
  clip[11] = e[8] * i[3] + e[9] * i[7] + e[10] * i[11] + e[11] * i[15];

  clip[12] = e[12] * i[0] + e[13] * i[4] + e[14] * i[8] + e[15] * i[12];
  clip[13] = e[12] * i[1] + e[13] * i[5] + e[14] * i[9] + e[15] * i[13];
  clip[14] = e[12] * i[2] + e[13] * i[6] + e[14] * i[10] + e[15] * i[14];
  clip[15] = e[12] * i[3] + e[13] * i[7] + e[14] * i[11] + e[15] * i[15];

  /* Extract the numbers for the RIGHT plane */
  frustum[0][0] = clip[3] - clip[0];
  frustum[0][1] = clip[7] - clip[4];
  frustum[0][2] = clip[11] - clip[8];
  frustum[0][3] = clip[15] - clip[12];

  /* Normalize the result */
  t = sqrt (frustum[0][0] * frustum[0][0] + frustum[0][1] * frustum[0][1] + frustum[0][2] * frustum[0][2]);
  frustum[0][0] /= t;
  frustum[0][1] /= t;
  frustum[0][2] /= t;
  frustum[0][3] /= t;

  /* Extract the numbers for the LEFT plane */
  frustum[1][0] = clip[3] + clip[0];
  frustum[1][1] = clip[7] + clip[4];
  frustum[1][2] = clip[11] + clip[8];
  frustum[1][3] = clip[15] + clip[12];

  /* Normalize the result */
  t = sqrt (frustum[1][0] * frustum[1][0] + frustum[1][1] * frustum[1][1] + frustum[1][2] * frustum[1][2]);
  frustum[1][0] /= t;
  frustum[1][1] /= t;
  frustum[1][2] /= t;
  frustum[1][3] /= t;

  /* Extract the BOTTOM plane */
  frustum[2][0] = clip[3] + clip[1];
  frustum[2][1] = clip[7] + clip[5];
  frustum[2][2] = clip[11] + clip[9];
  frustum[2][3] = clip[15] + clip[13];

  /* Normalize the result */
  t = sqrt (frustum[2][0] * frustum[2][0] + frustum[2][1] * frustum[2][1] + frustum[2][2] * frustum[2][2]);
  frustum[2][0] /= t;
  frustum[2][1] /= t;
  frustum[2][2] /= t;
  frustum[2][3] /= t;

  /* Extract the TOP plane */
  frustum[3][0] = clip[3] - clip[1];
  frustum[3][1] = clip[7] - clip[5];
  frustum[3][2] = clip[11] - clip[9];
  frustum[3][3] = clip[15] - clip[13];

  /* Normalize the result */
  t = sqrt (frustum[3][0] * frustum[3][0] + frustum[3][1] * frustum[3][1] + frustum[3][2] * frustum[3][2]);
  frustum[3][0] /= t;
  frustum[3][1] /= t;
  frustum[3][2] /= t;
  frustum[3][3] /= t;

  /* Extract the FAR plane */
  frustum[4][0] = clip[3] - clip[2];
  frustum[4][1] = clip[7] - clip[6];
  frustum[4][2] = clip[11] - clip[10];
  frustum[4][3] = clip[15] - clip[14];

  /* Normalize the result */
  t = sqrt (frustum[4][0] * frustum[4][0] + frustum[4][1] * frustum[4][1] + frustum[4][2] * frustum[4][2]);
  frustum[4][0] /= t;
  frustum[4][1] /= t;
  frustum[4][2] /= t;
  frustum[4][3] /= t;

  /* Extract the NEAR plane */
  frustum[5][0] = clip[3] + clip[2];
  frustum[5][1] = clip[7] + clip[6];
  frustum[5][2] = clip[11] + clip[10];
  frustum[5][3] = clip[15] + clip[14];

  /* Normalize the result */
  t = sqrt (frustum[5][0] * frustum[5][0] + frustum[5][1] * frustum[5][1] + frustum[5][2] * frustum[5][2]);
  frustum[5][0] /= t;
  frustum[5][1] /= t;
  frustum[5][2] /= t;
  frustum[5][3] /= t;
}

void
tgFrustum::ExtractFrustum ()
{
  float mv[16];
  float pj[16];

  /* Get the current PROJECTION matrix from OpenGL */
  glGetFloatv (GL_PROJECTION_MATRIX, pj);

  /* Get the current MODELVIEW matrix from OpenGL */
  glGetFloatv (GL_MODELVIEW_MATRIX, mv);

  ExtractFrustum (mat4 (pj), mat4 (mv));
}

bool
tgFrustum::PointInFrustum (float x, float y, float z)
{
  int p;

  for (p = 0; p < 6; p++)
  {
    if (frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= 0)
      return false;
  }

  return true;
}

bool
tgFrustum::SphereInFrustum (float x, float y, float z, float radius)
{
  int p;

  for (p = 0; p < 6; p++)
  {
    if (frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= -radius)
      return false;
  }
  return true;
}

void
tgFrustum::DrawFrustum ()
{
  // Get near and far from the Projection matrix.
  float near = m_intrinsic[14] / (m_intrinsic[10] - 1.0f);
  float far = m_intrinsic[14] / (1.0f + m_intrinsic[10]);

  // Get the sides of the near plane.
  const float nLeft = near * (m_intrinsic[2] - 1.0f) / m_intrinsic[0];
  const float nRight = near * (1.0f + m_intrinsic[2]) / m_intrinsic[0];
  const float nTop = near * (1.0f + m_intrinsic[6]) / m_intrinsic[5];
  const float nBottom = near * (m_intrinsic[6] - 1.0f) / m_intrinsic[5];

  // Get the sides of the far plane.
  const float fLeft = far * (m_intrinsic[2] - 1.0f) / m_intrinsic[0];
  const float fRight = far * (1.0f + m_intrinsic[2]) / m_intrinsic[0];
  const float fTop = far * (1.0f + m_intrinsic[6]) / m_intrinsic[5];
  const float fBottom = far * (m_intrinsic[6] - 1.0f) / m_intrinsic[5];

  /*
   0	glVertex3f(0.0f, 0.0f, 0.0f);
   1	glVertex3f(nLeft, nBottom, -near);
   2	glVertex3f(nRight, nBottom, -near);
   3	glVertex3f(nRight, nTop, -near);
   4	glVertex3f(nLeft, nTop, -near);
   5	glVertex3f(fLeft, fBottom, -far);
   6	glVertex3f(fRight, fBottom, -far);
   7	glVertex3f(fRight, fTop, -far);
   8	glVertex3f(fLeft, fTop, -far);
   */

  std::vector<vec3> v (10);
  v[0] = vec3 (nLeft, nBottom, near);
  v[1] = vec3 (nRight, nBottom, near);
  v[2] = vec3 (nRight, nTop, near);
  v[3] = vec3 (nLeft, nTop, near);

  v[4] = vec3 (fLeft, fBottom, far);
  v[5] = vec3 (fRight, fBottom, far);
  v[6] = vec3 (fRight, fTop, far);
  v[7] = vec3 (fLeft, fTop, far);

  v[8] = vec3 (0, 0, 0);
  v[9] = vec3 (0, 0, far);

  mat4 gl2cv;
  gl2cv[5] = -1.0;
  gl2cv[10] = -1.0;

  mat3 R = gl2cv * m_extrinsic;
  mat3 Rt = R.transpose ();

  vec3 t (m_extrinsic[12], -m_extrinsic[13], -m_extrinsic[14]);
  t = -(Rt * t);

  for (size_t i = 0; i < v.size (); i++)
    v[i] = Rt * v[i] + t;

  glColor3f (m_color.x, m_color.y, m_color.z);

  glBegin (GL_LINES);

  // cross far
//  glVertex3fv (v[4]);
//  glVertex3fv (v[6]);
//
//  glVertex3fv (v[5]);
//  glVertex3fv (v[7]);

  //near
  glVertex3fv (v[0]);
  glVertex3fv (v[1]);

  glVertex3fv (v[1]);
  glVertex3fv (v[2]);

  glVertex3fv (v[2]);
  glVertex3fv (v[3]);

  glVertex3fv (v[3]);
  glVertex3fv (v[0]);

  //far
  glVertex3fv (v[4]);
  glVertex3fv (v[5]);

  glVertex3fv (v[5]);
  glVertex3fv (v[6]);

  glVertex3fv (v[6]);
  glVertex3fv (v[7]);

  glVertex3fv (v[7]);
  glVertex3fv (v[4]);

  //sides
  glVertex3fv (v[0]);
  glVertex3fv (v[4]);

  glVertex3fv (v[1]);
  glVertex3fv (v[5]);

  glVertex3fv (v[2]);
  glVertex3fv (v[6]);

  glVertex3fv (v[3]);
  glVertex3fv (v[7]);

  // view ray
//  glVertex3fv (v[8]);
//  glVertex3fv (v[9]);

  glEnd ();
}

