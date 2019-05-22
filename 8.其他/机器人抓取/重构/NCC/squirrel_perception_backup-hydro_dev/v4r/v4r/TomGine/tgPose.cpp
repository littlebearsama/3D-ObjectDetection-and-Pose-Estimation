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

#include "tgPose.h"

#include <stdio.h>

#ifdef WIN32
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif

using namespace TomGine;

bool
tgPose::operator== (const tgPose &p) const
{
  return t == p.t && q == p.q;
}

tgPose
tgPose::operator* (const tgPose& p) const
{
  tgPose ret;
  mat3 R, pR, rR;
  vec3 t, pt, rt;

  this->GetPose (R, t);
  p.GetPose (pR, pt);

  rR = R * pR;
  rt = (R * pt) + t;

  ret.SetPose (rR, rt);
  return ret;
}

vec3
tgPose::operator* (const vec3& pt) const
{
  vec3 ret;
  mat3 R;
  vec3 t;

  this->GetPose (R, t);

  ret = (R * pt) + t;

  return ret;
}

tgPose
tgPose::operator+ (const tgPose &p) const
{
  tgPose res;
  res.t = t - p.t;
  res.q = q - p.q;
  return res;
}

tgPose
tgPose::operator- (const tgPose &p) const
{
  tgPose res;
  res.t = t - p.t;
  res.q = q - p.q;
  return res;
}

tgPose
tgPose::Transpose () const
{
  tgPose ret;
  mat3 R;
  vec3 t;
  GetPose (R, t);

  R = R.transpose ();
  t = -(R * t);

  ret.SetPose (R, t);
  return ret;
}

void
tgPose::Activate () const
{
  glPushMatrix ();
  glTranslatef (t.x, t.y, t.z);
  glMultMatrixf (q.getMatrix4 ());
}

void
tgPose::Deactivate () const
{
  glPopMatrix ();
}

void
tgPose::DrawCoordinates (float linelength, float linewidth) const
{
  this->Activate ();

  float l1 = linelength;
  glDisable (GL_LIGHTING);
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_TEXTURE_2D);
  glLineWidth (linewidth);

  glBegin (GL_LINES);
  glColor3f (1.0f, 0.0f, 0.0f);
  glVertex3f (0.0f, 0.0f, 0.0f);
  glVertex3f (l1, 0.0f, 0.0f);
  glColor3f (0.0f, 1.0f, 0.0f);
  glVertex3f (0.0f, 0.0f, 0.0f);
  glVertex3f (0.0f, l1, 0.0f);
  glColor3f (0.0f, 0.0f, 1.0f);
  glVertex3f (0.0f, 0.0f, 0.0f);
  glVertex3f (0.0f, 0.0f, l1);
  glEnd ();

  glEnable (GL_LIGHTING);
  glEnable (GL_DEPTH_TEST);

  this->Deactivate ();
}

void
tgPose::Print () const
{
  printf ("%f %f %f %f %f %f %f\n", t.x, t.y, t.z, q.x, q.y, q.z, q.w);
}

void
tgPose::SetPose (const mat3 &rot, const vec3 &pos)
{
  q.fromMatrix (rot);
  q.normalise ();

  t.x = pos.x;
  t.y = pos.y;
  t.z = pos.z;
}

void
tgPose::SetPose(const mat4 &p)
{
  mat3 R = p;
  vec3 t(p[12], p[13], p[14]);
  SetPose(R, t);
}

mat4
tgPose::GetMat4() const
{
  mat4 T = q.getMatrix4();

  T[12] = t.x;
  T[13] = t.y;
  T[14] = t.z;

  return T;
}

void
tgPose::GetPose (mat3 &rot, vec3 &pos) const
{

  rot = q.getMatrix3 ();

  pos.x = t.x;
  pos.y = t.y;
  pos.z = t.z;
}

mat3
tgPose::GetRotation () const
{
  return q.getMatrix3 ();
}

mat4
tgPose::GetPose () const
{

  mat3 Rt = q.getMatrix3 ().transpose ();
  mat4 E (Rt);

//  vec3 tn = Rt * t;

  E[3] = t.x;
  E[7] = t.y;
  E[11] = t.z;

  return E;
}

void
tgPose::Rotate (const float &x, const float &y, const float &z)
{
  if (x == 0.0 && y == 0.0 && z == 0.0)
    return;

  tgQuaternion q2;

  q2.fromEuler (x, y, z);
  q = q2 * q;
  q.normalise ();
}

void
tgPose::RotateAxis (const vec3 &r)
{
  // 	tgQuaternion q2;
  // 	q2.fromEuler(rot.x,rot.y,rot.z);
  // 	q = q2 * q;
  // 	q.normalise();
  vec3 r2 = r;
  tgQuaternion q2;
  float a = r2.length ();
  r2.normalize ();
  q2.fromAxis (r2, a);
  q = q2 * q;
  q.normalise ();
}

void
tgPose::RotateEuler (const vec3 &r)
{
  tgQuaternion q2;
  q2.fromEuler (r.x, r.y, r.z);
  q = q2 * q;
  q.normalise ();
}

void
tgPose::Translate (const float &x, const float &y, const float &z)
{
  t.x = t.x + x;
  t.y = t.y + y;
  t.z = t.z + z;
}

void
tgPose::Translate (const vec3 &v)
{
  t.x = t.x + v.x;
  t.y = t.y + v.y;
  t.z = t.z + v.z;
}
