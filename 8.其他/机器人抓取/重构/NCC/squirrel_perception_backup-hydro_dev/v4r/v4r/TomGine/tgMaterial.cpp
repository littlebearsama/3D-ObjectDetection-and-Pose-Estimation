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
#include "tgMaterial.h"

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

tgMaterial::tgMaterial()
{
  ambient = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  diffuse = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  specular = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  color = vec4(0.5f, 0.5f, 0.5f, 1.0f);
  shininess = 50.0f;
}

void tgMaterial::Activate() const
{
//  if(color.w < 1.0-epsilon)
//    glEnable(GL_BLEND);

  glEnable(GL_LIGHTING);

  Apply();
}

void tgMaterial::Apply() const
{
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
}

void tgMaterial::Deactivate() const
{
//  glDisable(GL_BLEND);
}

void tgMaterial::Color(float r, float g, float b, float a, float amb, float diff, float spec, float emis, float shiny)
{
  color = vec4(r, g, b, a);
  ambient = vec4(r, g, b, a) * amb;
  diffuse = vec4(r, g, b, a) * diff;
  specular = vec4(1.0f, 1.0f, 1.0f, a) * spec;
  emission = vec4(r, g, b, a) * emis;
  shininess = shiny;// * float(rand())/RAND_MAX;
}

void tgMaterial::Color(float Ra, float Ga, float Ba, float Aa,
                       float Rd, float Gd, float Bd, float Ad,
                       float Rs, float Gs, float Bs, float As,
                       float shiny)
{
  color = vec4(Rd, Gd, Bd, Ad);
  ambient = vec4(Ra, Ga, Ba, Aa);
  diffuse = vec4(Rd, Gd, Bd, Ad);
  specular = vec4(Rs, Gs, Bs, As);
  emission = vec4(0.0, 0.0, 0.0, 0.0);
  shininess = shiny;
}

void tgMaterial::Random(float brightness)
{
  vec4 c;
  c.random();
  c *= brightness;
  Color(c.x, c.y, c.z);
}

void tgMaterial::Exposure(float val)
{
  color *= val;
  ambient *= val;
  diffuse *= val;
  specular *= val;
  emission *= val;
  shininess *= val;
}


