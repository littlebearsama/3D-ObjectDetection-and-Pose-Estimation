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

#include "tgCamera.h"

using namespace TomGine;
using namespace std;

tgCamera::Parameter::Parameter()
{
  width = 640;
  height = 480;
  fx = 520.0f;
  fy = 520.0f;
  cx = 320.0f;
  cy = 240.0f;
  k1 = 0.0f;
  k2 = 0.0f;
  k3 = 0.0f;
  p1 = 0.0f;
  p2 = 0.0f;
  rot.fromRotVector(vec3(-2.0f, -1.0f, 0.5f));
  pos = vec3(0.6f, -0.2f, 0.5f);
  zNear = 0.01f;
  zFar = 4.0f;
}

mat4 tgCamera::Parameter::cv2intrinsic()
{
  // intrinsic parameters
  // transform the coordinate system of computer vision to OpenGL
  //   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
  //   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
  float _fx = 2.0f * fx / width; // scale range from [0 ... 640] to [0 ... 2]
  float _fy = 2.0f * fy / height; // scale range from [0 ... 480] to [0 ... 2]
  float _cx = 1.0 - (2.0f * cx / width); // move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float _cy = (2.0f * cy / height) - 1.0f; // flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float _z1 = (zFar + zNear) / (zNear - zFar); // entries for clipping planes
  float _z2 = 2.0f * zFar * zNear / (zNear - zFar); // look up for gluPerspective

  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0] = _fx;  intrinsic[4] = 0;  intrinsic[8] = _cx;  intrinsic[12] = 0;
  intrinsic[1] = 0;  intrinsic[5] = _fy;  intrinsic[9] = _cy;  intrinsic[13] = 0;
  intrinsic[2] = 0;  intrinsic[6] = 0;  intrinsic[10] = _z1;  intrinsic[14] = _z2;
  intrinsic[3] = 0;  intrinsic[7] = 0;  intrinsic[11] = -1;  intrinsic[15] = 0;
  // last row assigns w=-z which inverts cx and cy at w-division

  return intrinsic;
}

mat4 tgCamera::Parameter::cv2extrinsic()
{
  // computer vision camera coordinates to OpenGL camera coordinates transform
  // rotate PI about x-axis
  mat4 cv2gl;
  cv2gl[0] = 1.0;  cv2gl[4] = 0.0;   cv2gl[8] = 0.0;   cv2gl[12] = 0.0;
  cv2gl[1] = 0.0;  cv2gl[5] = -1.0;  cv2gl[9] = 0.0;   cv2gl[13] = 0.0;
  cv2gl[2] = 0.0;  cv2gl[6] = 0.0;   cv2gl[10] = -1.0; cv2gl[14] = 0.0;
  cv2gl[3] = 0.0;  cv2gl[7] = 0.0;   cv2gl[11] = 0.0;  cv2gl[15] = 1.0;

  // extrinsic parameters
  // look up comments in tools/hardware/video/src/slice/Video.ice
  // p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
  mat4 extrinsic;
  extrinsic[0] = rot[0];  extrinsic[4] = rot[3];  extrinsic[8] = rot[6];  extrinsic[12] = 0.0;
  extrinsic[1] = rot[1];  extrinsic[5] = rot[4];  extrinsic[9] = rot[7];  extrinsic[13] = 0.0;
  extrinsic[2] = rot[2];  extrinsic[6] = rot[5];  extrinsic[10] = rot[8]; extrinsic[14] = 0.0;
  extrinsic[3] = 0.0;   extrinsic[7] = 0.0;   extrinsic[11] = 0.0;  extrinsic[15] = 1.0;

  // 	extrinsic = extrinsic.transpose();							// R^T
  //  vec4 tp = vec4(t.x, t.y, t.z, 1.0);
//  vec4 tp = -(extrinsic * vec4(pos.x, pos.y, pos.z, 1.0)); // -R^T*t
  extrinsic[12] = pos.x;
  extrinsic[13] = pos.y;
  extrinsic[14] = pos.z;
  extrinsic = cv2gl * extrinsic;

  return extrinsic;
}

namespace TomGine
{
std::ostream& operator << (std::ostream& os, tgCamera::Parameter& param)
{
  os << endl;
  os << "# ----- Camera Parameter ----- #" << endl;
  os << "# resolution" << endl;
  os << param.width << " " << param.height << endl;

  os << "# intrinsics" << endl;
  os << param.fx << " " << param.fy << " " << param.cx << " " << param.cy << endl;

  os << "# radial distortion parameters" << endl;
  os << param.k1 << " " << param.k2 << " " << param.k3 << endl;

  os << "# tangential distortion parameters" << endl;
  os << param.p1 << " " << param.p2 << endl;

  os << "# extrinsics rot" << endl;
  os << param.rot[0] << " " << param.rot[3] << " " << param.rot[6] << endl;
  os << param.rot[1] << " " << param.rot[4] << " " << param.rot[7] << endl;
  os << param.rot[2] << " " << param.rot[5] << " " << param.rot[8] << endl;

  os << "# extrinsics pos" << endl;
  os << param.pos[0] << " " << param.pos[1] << " " << param.pos[2] << endl;

  os << "# Clipping planes of virtual camera" << endl;
  os << param.zNear << " " << param.zFar << endl;

  return os;
}


std::istream& operator >> (std::istream& is, tgCamera::Parameter& param)
{
  string linebuffer;

  getline(is,linebuffer);
  getline(is,linebuffer);

  getline(is,linebuffer);
  is >> param.width;
  is >> param.height;
  is.get();

  getline(is,linebuffer);
  is >> param.fx;
  is >> param.fy;
  is >> param.cx;
  is >> param.cy;
  is.get();

  getline(is,linebuffer);
  is >> param.k1;
  is >> param.k2;
  is >> param.k3;
  is.get();

  getline(is,linebuffer);
  is >> param.p1;
  is >> param.p2;
  is.get();

  getline(is,linebuffer);
  is >> param.rot[0]; is >> param.rot[3]; is >> param.rot[6]; is.get();
  is >> param.rot[1]; is >> param.rot[4]; is >> param.rot[7]; is.get();
  is >> param.rot[2]; is >> param.rot[5]; is >> param.rot[8]; is.get();
  is.get();

  getline(is,linebuffer);
  is >> param.pos[0]; is >> param.pos[1]; is >> param.pos[2]; is.get();
  is.get();

  getline(is,linebuffer);
  is >> param.zNear;
  is >> param.zFar;
  is.get();

  return is;
}
}

void tgCamera::Parameter::print(){
  printf("%d %d\n", width, height);
  printf("%f %f\n", fx, fy);
  printf("%f %f\n", cx, cy);
  printf("%f %f %f\n", k1, k2, k3);
  printf("%f %f\n", p1, p2);
  printf("\n %f %f %f\n", rot[0], rot[1], rot[2]);
  printf(" %f %f %f\n", rot[3], rot[4], rot[5]);
  printf(" %f %f %f\n", rot[6], rot[7], rot[8]);
  printf("\n %f %f %f\n", pos.x, pos.y, pos.z);
  printf("%f %f\n", zNear, zFar);
}

tgCamera::tgCamera()
{
  m_width = 640;
  m_height = 480;

  this->SetIntrinsicCV(640,480,320,240,0.01,100.0);
  f = vec3(0.0f, 0.0f, -1.0f);
  s = vec3(1.0f, 0.0f, 0.0f);
  u = vec3(0.0f, 1.0f, 0.0f);
  fsu2pvu();
  fsu2extrinsic();
  ApplyTransform();
}

tgCamera tgCamera::Set(mat3 intrinsic, mat3 R, vec3 T, unsigned width, unsigned height, float near, float far)
{
  TomGine::tgCamera::Parameter param;
  param.width = width;
  param.height = height;
  param.zNear = near;
  param.zFar = far;

  // Instrinsic parameters:
  // entries of the camera matrix
  param.fx = intrinsic[0];
  param.fy = intrinsic[4];
  param.cx = intrinsic[2];
  param.cy = intrinsic[5];

  // radial distortion parameters
  param.k1 = 0.0;
  param.k2 = 0.0;
  param.k3 = 0.0;
  // tangential distortion parameters
  param.p1 = 0.0;
  param.p2 = 0.0;

  param.rot = R;
  param.pos = T;

  tgCamera cam;
  cam.Set(param);
  return cam;
}

void tgCamera::Set(tgCamera::Parameter camPars)
{

  m_width = camPars.width;
  m_height = camPars.height;
  m_zFar = camPars.zFar;
  m_zNear = camPars.zNear;

  // intrinsic parameters
  // transform the coordinate system of computer vision to OpenGL
  //   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
  //   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
  float fx = 2.0f * camPars.fx / camPars.width; // scale range from [0 ... 640] to [0 ... 2]
  float fy = 2.0f * camPars.fy / camPars.height; // scale range from [0 ... 480] to [0 ... 2]
  float cx = 1.0f - (2.0f * camPars.cx / camPars.width); // move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = (2.0f * camPars.cy / camPars.height) - 1.0f; // flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (camPars.zFar + camPars.zNear) / (camPars.zNear - camPars.zFar); // entries for clipping planes
  float z2 = 2.0f * camPars.zFar * camPars.zNear / (camPars.zNear - camPars.zFar); // look up for gluPerspective

  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0] = fx;  intrinsic[4] = 0;  intrinsic[8] = cx;  intrinsic[12] = 0;
  intrinsic[1] = 0;  intrinsic[5] = fy;  intrinsic[9] = cy;  intrinsic[13] = 0;
  intrinsic[2] = 0;  intrinsic[6] = 0;  intrinsic[10] = z1;  intrinsic[14] = z2;
  intrinsic[3] = 0;  intrinsic[7] = 0;  intrinsic[11] = -1;  intrinsic[15] = 0;
  // last row assigns w=-z which inverts cx and cy at w-division

  // computer vision camera coordinates to OpenGL camera coordinates transform
  // rotate 180� about x-axis
  mat4 cv2gl;
  cv2gl[0] = 1.0;  cv2gl[4] = 0.0;   cv2gl[8] = 0.0;   cv2gl[12] = 0.0;
  cv2gl[1] = 0.0;  cv2gl[5] = -1.0;  cv2gl[9] = 0.0;   cv2gl[13] = 0.0;
  cv2gl[2] = 0.0;  cv2gl[6] = 0.0;   cv2gl[10] = -1.0; cv2gl[14] = 0.0;
  cv2gl[3] = 0.0;  cv2gl[7] = 0.0;   cv2gl[11] = 0.0;  cv2gl[15] = 1.0;

  // extrinsic parameters
  // look up comments in tools/hardware/video/src/slice/Video.ice
  // p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
  mat3 R = camPars.rot;
  vec3 t = camPars.pos;
  mat4 extrinsic;
  extrinsic[0] = R[0];  extrinsic[4] = R[3];  extrinsic[8] = R[6];  extrinsic[12] = 0.0;
  extrinsic[1] = R[1];  extrinsic[5] = R[4];  extrinsic[9] = R[7];  extrinsic[13] = 0.0;
  extrinsic[2] = R[2];  extrinsic[6] = R[5];  extrinsic[10] = R[8]; extrinsic[14] = 0.0;
  extrinsic[3] = 0.0;   extrinsic[7] = 0.0;   extrinsic[11] = 0.0;  extrinsic[15] = 1.0;

  // 	extrinsic = extrinsic.transpose();							// R^T
  //  vec4 tp = vec4(t.x, t.y, t.z, 1.0);
  vec4 tp = -(extrinsic * vec4(t.x, t.y, t.z, 1.0)); // -R^T*t
  extrinsic[12] = tp.x;
  extrinsic[13] = tp.y;
  extrinsic[14] = tp.z;
  extrinsic = cv2gl * extrinsic;

  // set camera parameters
  SetViewport(camPars.width, camPars.height);
  SetZRange(camPars.zNear, camPars.zFar);
  SetIntrinsic(intrinsic);
  SetExtrinsic(extrinsic);
  SetPos(camPars.pos.x, camPars.pos.y, camPars.pos.z);
  m_frustum.ExtractFrustum(intrinsic, extrinsic);
}

void tgCamera::Set(vec3 pos, vec3 view, vec3 up, float fovy, unsigned width, unsigned height, float zNear, float zFar,
                   tgCamera::Projection proj)
{
  this->pos = vec3(pos.x, pos.y, pos.z); // set position
  this->view = vec3(view.x, view.y, view.z); // set view point
  this->up = vec3(up.x, up.y, up.z); // set the up vector
  pvu2fsu();

  m_fovy = fovy;
  m_width = width;
  m_height = height;
  m_zNear = zNear;
  m_zFar = zFar;
  m_projection = proj;

  fwh2intrinsic();
  fsu2extrinsic();
}

void tgCamera::SetExtrinsic(float* M)
{
  m_extrinsic = mat4(M);

  extrinsic2fsu();
  fsu2pvu();
}

void tgCamera::SetIntrinsic(float* M)
{
  m_intrinsic = mat4(M);

  intrinsic2fwh();
}

void tgCamera::SetIntrinsicCV(float _fx, float _fy, float _cx, float _cy, float _zNear, float _zFar)
{
  m_zFar = _zFar;
  m_zNear = _zNear;

  float fx = 2.0f * _fx / m_width; // scale range from [0 ... m_width] to [0 ... 2]
  float fy = 2.0f * _fy / m_height; // scale range from [0 ... m_height] to [0 ... 2]
  float cx = 1.0f - (2.0f * _cx / m_width); // move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = (2.0f * _cy / m_height) - 1.0f; // flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (_zFar + _zNear) / (_zNear - _zFar); // entries for clipping planes
  float z2 = 2.0f * _zFar * _zNear / (_zNear - _zFar); // look up for gluPerspective

  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0] = fx;  intrinsic[4] = 0;  intrinsic[8] = cx;  intrinsic[12] = 0;
  intrinsic[1] = 0;  intrinsic[5] = fy;  intrinsic[9] = cy;  intrinsic[13] = 0;
  intrinsic[2] = 0;  intrinsic[6] = 0;  intrinsic[10] = z1;  intrinsic[14] = z2;
  intrinsic[3] = 0;  intrinsic[7] = 0;  intrinsic[11] = -1;  intrinsic[15] = 0;

  m_intrinsic = intrinsic;
  intrinsic2fwh();
}

void tgCamera::SetIntrinsic(float fovy, unsigned width, unsigned height, float zNear, float zFar,
                            unsigned short projection)
{
  m_fovy = fovy;
  m_width = width;
  m_height = height;
  m_zNear = zNear;
  m_zFar = zFar;
  m_projection = projection;

  fwh2intrinsic();
}

void tgCamera::SetViewport(unsigned w, unsigned h)
{
  m_width = w;
  m_height = h;
}

void tgCamera::SetZRange(float _near, float _far)
{
  m_zNear = _near;
  m_zFar = _far;
  fwh2intrinsic();
}

vec2 tgCamera::ToImageSpace(const vec3 &world_space) const
{
  vec2 ret;
  vec4 tmp = m_intrinsic * m_extrinsic * vec4(world_space, 1.0f);

  tmp.x = tmp.x / tmp.w;
  tmp.y = tmp.y / tmp.w;

  ret.x = (tmp.x + 1.0f) * 0.5f * m_width;
  ret.y = (tmp.y + 1.0f) * 0.5f * m_height;

  return ret;
}

void tgCamera::Activate()
{
  // Apply intrinsic parameters
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(m_intrinsic);

  // Apply extrinsic parameters
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(m_extrinsic);

  glViewport(0, 0, m_width, m_height);

  // Extract frustum planes
  m_frustum.ExtractFrustum();
}

void tgCamera::Print() const
{

  mat4 m = m_extrinsic;
  mat4 p = m_intrinsic;
  printf("Viewport: %u %u\n", m_width, m_height);
  printf("Modelview matrix:\n");
  m.print();
  printf("Projection matrix:\n");
  p.print();
}

void tgCamera::pvu2fsu()
{
  f = view - pos;
  f.normalize();
  s = cross(f, up);
  s.normalize();
  u = cross(s, f);
  u.normalize();
}

void tgCamera::fsu2pvu()
{
  view = pos + f;
  up = u;
}

void tgCamera::fsu2extrinsic()
{
  float fR[16] = { s.x, u.x, -f.x, 0.0, s.y, u.y, -f.y, 0.0, s.z, u.z, -f.z, 0.0, 0.0, 0.0, 0.0, 1.0 };

  float ft[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -pos.x, -pos.y, -pos.z, 1.0 };
  mat4 R(fR);
  mat4 t(ft);

  m_extrinsic = R * t;
}

void tgCamera::extrinsic2fsu()
{
  s.x = m_extrinsic[0];
  u.x = m_extrinsic[1];
  f.x = -m_extrinsic[2];
  s.y = m_extrinsic[4];
  u.y = m_extrinsic[5];
  f.y = -m_extrinsic[6];
  s.z = m_extrinsic[8];
  u.z = m_extrinsic[9];
  f.z = -m_extrinsic[10];

  float fR[16] = { s.x, u.x, -f.x, 0.0, s.y, u.y, -f.y, 0.0, s.z, u.z, -f.z, 0.0, 0.0, 0.0, 0.0, 1.0 };
  mat4 R(fR);
  mat4 t;

  t = R.inverse() * m_extrinsic;

  pos.x = -t[12];
  pos.y = -t[13];
  pos.z = -t[14];
}

void tgCamera::fwh2intrinsic()
{
  if (m_projection == GL_ORTHO)
  {
    m_intrinsic.identity();
    m_intrinsic[0] = 2.0f / float(m_width);
    m_intrinsic[5] = 2.0f / float(m_height);
    m_intrinsic[10] = -2.0f / (m_zFar - m_zNear);
    m_intrinsic[12] = -1.0f;
    m_intrinsic[13] = -1.0f;
    m_intrinsic[14] = -(m_zFar + m_zNear) / (m_zFar - m_zNear);

  } else if (m_projection == GL_PERSPECTIVE)
  {
    float f = 1.0f / tan(m_fovy*0.5);
    float a = float(m_width) / float(m_height);
    m_intrinsic.zero();
    m_intrinsic[0] = f / a;
    m_intrinsic[5] = f;
    m_intrinsic[10] = (m_zFar + m_zNear) / (m_zNear - m_zFar);
    m_intrinsic[11] = -1.0f;
    m_intrinsic[14] = (m_zFar * m_zNear * 2.0) / (m_zNear - m_zFar);
  }
}

void tgCamera::intrinsic2fwh()
{
  if (m_intrinsic[11] == -1.0)
  {
    const float &z1 = m_intrinsic[10];
    const float &z2 = m_intrinsic[14];
    const float &fy = m_intrinsic[5];

    m_zFar = z2 / (z1 + 1.0f);
    m_zNear = z2 * m_zFar / (z2 - 2.0f * m_zFar);

    m_fovy = 2.0f * atan(1 / fy) * 180.0 / M_PI;

  } else
  {
    printf("[tgCamera::intrinsic2fwh] Warning, function not implemented for orthonormal projection\n");
  }
}

tgPose tgCamera::GetPose() const
{
  printf("[tgCamera::GetPose()] Not implemented yet\n");
  return tgPose();
}

vec2 tgCamera::GetTexCoords(vec3 point) const
{
  TomGine::vec4 tmp = m_intrinsic * (m_extrinsic * vec4(point, 1.0f));

  TomGine::vec2 texcoords;
  texcoords.x = (tmp.x / tmp.w + 1.0) * 0.5;
  texcoords.y = (tmp.y / tmp.w + 1.0) * 0.5;

  return texcoords;
}

vec2 tgCamera::ProjectInto(vec3 point) const
{
  vec4 tmp = m_intrinsic * (m_extrinsic * vec4(point, 1.0f));
  vec2 p;
  p.x = tmp.x / tmp.w;
  p.y = tmp.y / tmp.w;

  if (this->f * point < 0.0f)
  { // avoids backprojection
    p.x = -p.x;
    p.y = -p.y;
  }

  p.x = round(m_width * (p.x + 1.0f) * 0.5f);
  p.y = round(m_height * (p.y + 1.0f) * 0.5f);

  return p;
}

void tgCamera::GetViewRay(int u, int v, vec3 &start, vec3 &dir) const
{
  //  unsigned wd2(m_width>>1);
  //  unsigned hd2(m_height>>1);
  //  int du = u - wd2;
  //  int dv = v - hd2;
  //dir.x = (float(du) - m_intrinsic[8] * wd2) / (m_intrinsic[0] * wd2);
  //dir.y = (float(dv) - m_intrinsic[9] * hd2) / (m_intrinsic[5] * hd2);
  dir.x = (2.0 * float(u) / m_width - 1.0f + m_intrinsic[8]) / m_intrinsic[0];
  dir.y = (2.0 * float(v) / m_height - 1.0f + m_intrinsic[9]) / m_intrinsic[5];
  dir.z = 1.0f;
  dir.normalize();
  mat4 mi = m_extrinsic.inverse();
  dir.y = -dir.y;
  dir.z = -dir.z;
  dir = mi * vec4(dir.x, dir.y, dir.z, 0.0);
  start = vec3(mi[12], mi[13], mi[14]);
}

//****************************************************************************
// Translations
void tgCamera::Translate(vec3 v)
{
  pos = pos + v;
  view = pos + f;
}

void tgCamera::Translate(float x, float y, float z, float fWay)
{
  vec3 v = vec3(x, y, z);
  v.normalize();
  pos = pos + v * fWay;
  view = pos + f;
}

void tgCamera::TranslateF(float fWay)
{
  pos = pos + f * fWay;
  view = pos + f;
}

void tgCamera::TranslateS(float fWay)
{
  pos = pos + s * fWay;
  view = pos + f;
}

void tgCamera::TranslateU(float fWay)
{
  pos = pos + u * fWay;
  view = pos + f;
}

//****************************************************************************
// Rotations
void tgCamera::Rotate(float x, float y, float z, float fAngle)
{
  vec3 v = vec3(x, y, z);
  f.rotate(fAngle, v);
  s.rotate(fAngle, v);
  u.rotate(fAngle, v);
  fsu2pvu();
}

void tgCamera::RotateF(float fAngle)
{
  s.rotate(fAngle, f);
  u = cross(s, f);
  u.normalize();
  fsu2pvu();
}

void tgCamera::RotateS(float fAngle)
{
  f.rotate(fAngle, s);
  u = cross(s, f);
  u.normalize();
  fsu2pvu();
}

void tgCamera::RotateU(float fAngle)
{
  f.rotate(fAngle, u);
  s = cross(f, u);
  s.normalize();
  fsu2pvu();
}

//void tgCamera::RotateX(float fAngle){
//	printf("tgCamera.RotateX not implemented! \n");
//}

void tgCamera::RotateY(float fAngle)
{
  vec3 y = vec3(0.0f, 1.0f, 0.0f);
  f.rotate(fAngle, y);
  s = cross(f, y);
  s.normalize();
  u = cross(s, f);
  u.normalize();
  fsu2pvu();
}

//void tgCamera::RotateZ(float fAngle){
//	printf("tgCamera.RotateZ not implemented! \n");
//}

void tgCamera::Orbit(vec3 vPoint, vec3 vAxis, float fAngle)
{
  vec3 d = pos - vPoint;

  d.rotate(fAngle, vAxis);
  pos = vPoint + d;

  f.rotate(fAngle, vAxis);
  s.rotate(fAngle, vAxis);
  u.rotate(fAngle, vAxis);
}

void tgCamera::LookAt(const vec3 &pov)
{
  this->view = pov;
  pvu2fsu();
}

void tgCamera::LookAt(const vec3 &pos, const vec3 &pov, const vec3& up)
{
  this->pos = pos;
  this->view = pov;
  this->up = up;
  pvu2fsu();
}

//****************************************************************************
// Movement
void tgCamera::ApplyTransform()
{
  fsu2extrinsic();
}

