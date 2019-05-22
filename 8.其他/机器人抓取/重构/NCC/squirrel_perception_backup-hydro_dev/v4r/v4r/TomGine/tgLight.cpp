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

#include "tgLight.h"

using namespace TomGine;

tgLight::tgLight()
{
  ambient = vec4(0.4f, 0.4f, 0.4f, 1.0f);
  diffuse = vec4(1.0f, 1.0f, 1.0f, 1.0f);
  specular = vec4(1.0f, 1.0f, 1.0f, 1.0f);
  position = vec4(0.0f, 0.0f, 1.0f, 0.0f);
}

void tgLight::Activate(int id) const
{
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0 + id, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0 + id, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0 + id, GL_SPECULAR, specular);
  glLightfv(GL_LIGHT0 + id, GL_POSITION, position);
  glEnable(GL_LIGHT0 + id);
}

void tgLight::Color(float r, float g, float b, float a)
{
  ambient = vec4(r, g, b, a) * 0.5f;
  diffuse = vec4(0.2f, 0.2f, 0.2f, a) + vec4(r, g, b, 0.0f) * 0.8f;
  specular = vec4(0.3f, 0.3f, 0.3f, a);
  //	position = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void tgLight::Random()
{
  vec4 c;
  c.random();
  Color(c.x, c.y, c.z);
}

void tgLight::Draw()
{
  glEnable(GL_POINT_SMOOTH);
  glDisable(GL_LIGHTING);
  glPointSize(10.0f);
  glLineWidth(2.0f);

  if (position.w == 1.0f)
  {
    glColor3fv(diffuse);
    glBegin(GL_POINTS);
    glVertex3fv(position);
    glEnd();
  } else
  {
    glColor3fv(diffuse);
    glBegin(GL_POINTS);
    glVertex3fv(position);
    glEnd();

    glBegin(GL_LINES);
    glVertex3fv(-position);
    glVertex3fv(position);
    glEnd();
  }

  glPointSize(1.0f);
  glLineWidth(1.0f);
}

