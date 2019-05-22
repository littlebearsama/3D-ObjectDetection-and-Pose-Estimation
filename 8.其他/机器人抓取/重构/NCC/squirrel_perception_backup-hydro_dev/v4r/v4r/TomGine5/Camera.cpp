/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Thomas Moerwald
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

#include "Camera.h"

#include <GL/gl.h>
#include <iostream>
#include <stdio.h>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace tg;

Camera::Camera()
{
  m_intrinsic = glm::perspective<float>(45.0f*M_PI/180.0f, 4.0f/3.0f, 0.1f, 10.0f);
  m_extrinsic = glm::translate(glm::mat4(1.0f), glm::vec3(0,0,-1.0f));
}

void Camera::Print()
{
  printf("Intrinsic:\n");
  const glm::mat4& I = m_intrinsic;
  for (size_t i=0; i<4; i++)
    printf("  %f %f %f %f\n", I[0][i], I[1][i], I[2][i], I[3][i]);
  //    std::cout << I[0][i] << " " << I[1][i] << " " << I[2][i] << " " << I[3][i] << std::endl;

  printf("Extrinsic:\n");
  const glm::mat4& E = m_extrinsic;
  for (size_t i=0; i<4; i++)
    printf("  %f %f %f %f\n", E[0][i], E[1][i], E[2][i], E[3][i]);
  //    std::cout << E[0][i] << " " << E[1][i] << " " << E[2][i] << " " << E[3][i] << std::endl;
}

float Camera::GetFar() const
{
  const float &z1 = m_intrinsic[2][2];
  const float &z2 = m_intrinsic[2][3];

  return (z2 / (z1 + 1.0f));
}

float Camera::GetNear() const
{
  const float &z1 = m_intrinsic[2][2];
  const float &z2 = m_intrinsic[2][3];

  float far = z2 / (z1 + 1.0f);
  return (z2 * far / (z2 - 2.0f * far));
}

glm::vec3 Camera::GetForward() const
{
  return glm::vec3(m_extrinsic[0][2], m_extrinsic[1][2], m_extrinsic[2][2]);
}

glm::vec3 Camera::GetSideward() const
{
  return glm::vec3(m_extrinsic[0][0], m_extrinsic[1][0], m_extrinsic[2][0]);
}

glm::vec3 Camera::GetUpward() const
{
  return glm::vec3(m_extrinsic[0][1], m_extrinsic[1][1], m_extrinsic[2][1]);
}

void Camera::SetPerspective(float fx, float fy, float cx, float cy,
                            float width, float height, float near, float far)
{
  fx = 2.0f * fx / width;
  fy = 2.0f * fy / height;
  cx = 1.0f - (2.0f * cx / width);
  cy = (2.0f * cy / height) - 1.0f;
  float z1 = (far + near) / (near - far);
  float z2 = 2.0f * far * near / (near - far);

  m_intrinsic[0][0] = fx; m_intrinsic[1][0] = 0;  m_intrinsic[2][0] = cx;  m_intrinsic[3][0] = 0;
  m_intrinsic[0][1] = 0;  m_intrinsic[1][1] = fy; m_intrinsic[2][1] = cy;  m_intrinsic[3][1] = 0;
  m_intrinsic[0][2] = 0;  m_intrinsic[1][2] = 0;  m_intrinsic[2][2] = z1;  m_intrinsic[3][2] = z2;
  m_intrinsic[0][3] = 0;  m_intrinsic[1][3] = 0;  m_intrinsic[2][3] = -1;  m_intrinsic[3][3] = 0;
}

void Camera::SetOrtho(float width, float height, float near, float far)
{
  m_intrinsic = glm::mat4(1.0f);
  m_intrinsic[0][0] = 2.0f / float(width);
  m_intrinsic[1][1] = 2.0f / float(height);
  m_intrinsic[2][2] = -2.0f / (far - near);
  m_intrinsic[3][0] = -1.0f;
  m_intrinsic[3][1] = -1.0f;
  m_intrinsic[3][2] = -(far + near) / (far - near);
}

void Camera::SetExtrinsic(const glm::mat4& E)
{
  m_extrinsic = E;
}


glm::mat4 Camera::cv2gl(const glm::mat4& cvext)
{
  glm::mat4 T(1.0f);
  T[1][1] = -1.0f;
  T[2][2] = -1.0f;
  return T*cvext;
}

void Camera::TranslateForward(float d)
{
  glm::vec3 f(m_extrinsic[0][2], m_extrinsic[1][2], m_extrinsic[2][2]);
  m_extrinsic = glm::translate(m_extrinsic, f*d);
}

void Camera::TranslateSideward(float d)
{
  glm::vec3 s(m_extrinsic[0][0], m_extrinsic[1][0], m_extrinsic[2][0]);
  m_extrinsic = glm::translate(m_extrinsic, s*d);
}

void Camera::TranslateUpward(float d)
{
  glm::vec3 u(m_extrinsic[0][1], m_extrinsic[1][1], m_extrinsic[2][1]);
  m_extrinsic = glm::translate(m_extrinsic, u*d);
}

void Camera::Orbit(glm::vec3 point, glm::vec3 axis, float angle)
{
  m_extrinsic = glm::translate(m_extrinsic, point);
  m_extrinsic = glm::rotate(m_extrinsic, -angle, axis);
  m_extrinsic = glm::translate(m_extrinsic, -point);
}

void Camera::Activate() const
{
  // Apply intrinsic parameters
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(glm::value_ptr(m_intrinsic));

  // Apply extrinsic parameters
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(glm::value_ptr(m_extrinsic));
}


