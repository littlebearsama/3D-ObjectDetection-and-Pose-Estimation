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

#ifndef _TG5_CAMERA_H_
#define _TG5_CAMERA_H_

#include <glm/glm.hpp>

namespace tg{

class Camera
{

private:
  glm::mat4 m_extrinsic;
  glm::mat4 m_intrinsic;

public:
  Camera();

  void Print();

  float GetFar() const;
  float GetNear() const;

  glm::vec3 GetForward() const;
  glm::vec3 GetSideward() const;
  glm::vec3 GetUpward() const;

  void SetPerspective(float fx, float fy, float cx, float cy,
                      float width, float height, float near, float far);

  void SetOrtho(float width, float height, float near, float far);

  void SetExtrinsic(const glm::mat4& E);
  static glm::mat4 cv2gl(const glm::mat4& cvext);

  void TranslateForward(float d);
  void TranslateSideward(float d);
  void TranslateUpward(float d);

  void Orbit(glm::vec3 point, glm::vec3 axis, float angle);

  void Activate() const;
};


}

#endif
