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

#include "tgPlot2D.h"
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

tgPlot2D::tgPlot2D(int x, int y, unsigned w, unsigned h)
{
  Define(x, y, w, h);
  m_fontcolor = vec3(0.5f, 0.5f, 0.5f);
  m_axiscolor = vec3(0.5f, 0.5f, 0.5f);
  m_gridcolor = vec3(0.5f, 0.5f, 0.5f);

  m_y1_color = vec3(1.0f, 1.0f, 0.0f);
  m_y2_color = vec3(1.0f, 0.0f, 1.0f);
  m_y3_color = vec3(0.0f, 1.0f, 1.0f);
  m_y4_color = vec3(0.0f, 1.0f, 0.0f);
}

void tgPlot2D::Define(int x, int y, unsigned w, unsigned h)
{
  this->x = x;
  this->y = y;
  this->w = w;
  this->h = h;

  m_fontsize = 14;

  m_buffer_size = w;

  x_min = 0.0f;
  x_max = 1.0f;
  y_min = 0.0f;
  y_max = 1.0f;
  x_scale = w;
  y_scale = h;
}

void tgPlot2D::Reset()
{
  m_buffer_x.clear();
  m_buffer_y_1.clear();
  m_buffer_y_2.clear();
  m_buffer_y_3.clear();
  m_buffer_y_4.clear();

  m_timer.Reset();
}

float tgPlot2D::findmax(const std::vector<float> &x) const
{
  if( x.empty() )
    return 0.0f;

  float fmax = 0.0f;
  for( unsigned i = 0; i < x.size(); i++ )
    if( fmax < x[i] )
      fmax = x[i];

  return fmax;
}

float tgPlot2D::findmin(const std::vector<float> &x) const
{
  if( x.empty() )
    return 0.0f;

  float fmin = x[0];
  for( unsigned i = 1; i < x.size(); i++ )
    if( x[i] < fmin )
      fmin = x[i];

  return fmin;
}

void tgPlot2D::updateBuffer(float y1, float y2, float y3, float y4)
{
  m_timer.Update();

  float t = float(m_timer.GetApplicationTime());

  // Add value only if it satisfies buffer size
  if( !m_buffer_x.empty() )
  {
    float x = m_buffer_x[m_buffer_x.size() - 1];
    float d = floor((t - x) * x_scale);
    if( d < float(w) / (m_buffer_size) )
      return;
  }

  float x_max = findmax(m_buffer_x);
  float x_min = findmin(m_buffer_x);
  float x_range = x_max - x_min;

  if( x_range > (this->x_max - this->x_min) || m_buffer_x.size() >= m_buffer_size )
  {
    this->Reset();
    t = 0.0f;
  }

  m_buffer_x.push_back(t);
  m_buffer_y_1.push_back(y1);
  m_buffer_y_2.push_back(y2);
  m_buffer_y_3.push_back(y3);
  m_buffer_y_4.push_back(y4);
}

void tgPlot2D::Axis(float x_min, float x_max, float y_min, float y_max)
{
  this->x_min = x_min;
  this->x_max = x_max;
  this->y_min = y_min;
  this->y_max = y_max;

  this->x_scale = float(w) / (x_max - x_min);
  this->y_scale = float(h) / (y_max - y_min);
}

void tgPlot2D::DrawAxis(bool lables, bool grid)
{
  float fx = float(x);
  float fy = float(y);
  float fw = float(w);
  float fh = float(h);

  float num_lines_h = 10.0f;
  float num_lines_v = 10.0f;

  // Draw Grid
  if( grid )
  {
    glLineWidth(1.0f);
    glColor3f(m_gridcolor.x, m_gridcolor.y, m_gridcolor.z);
    glBegin(GL_LINES);
    for( unsigned i = 1; i <= num_lines_h; i++ )
    {
      glVertex3f(fx, fy + (fh / num_lines_h) * i, 0.0);
      glVertex3f(fx + fw, fy + (fh / num_lines_h) * i, 0.0);
    }
    for( unsigned i = 1; i <= num_lines_v; i++ )
    {
      glVertex3f(fx + (fw / num_lines_v) * i, fy, 0.0);
      glVertex3f(fx + (fw / num_lines_v) * i, fy + fh, 0.0);
    }
    glEnd();
  }

  // Draw number on axis
  if( lables )
  {
    for( unsigned i = 0; i <= num_lines_h; i++ )
    {
      char b[8];
      sprintf(b, "%.2f", y_min + ((y_max - y_min) / num_lines_h) * i);
      g_font->Print(b, m_fontsize, int(fx - 40), int(fy + (fh / num_lines_h) * i - 5), m_fontcolor.x, m_fontcolor.y,
          m_fontcolor.z);
    }
    for( unsigned i = 0; i <= num_lines_v; i++ )
    {
      char b[8];
      sprintf(b, "%.1f", x_min + ((x_max - x_min) / num_lines_v) * i);
      g_font->Print(b, m_fontsize, int(fx + (fw / num_lines_v) * i - 10), int(fy - 20), m_fontcolor.x, m_fontcolor.y,
          m_fontcolor.z);
    }
  }

  // Draw axis
  glLineWidth(2.0f);
  glColor3f(m_axiscolor.x, m_axiscolor.y, m_axiscolor.z);
  glBegin(GL_LINES);
  glVertex3f(fx, fy, 0.0);
  glVertex3f(fx + fw, fy, 0.0);
  glVertex3f(fx, fy, 0.0);
  glVertex3f(fx, fy + fh, 0.0);
  glEnd();
}

void tgPlot2D::DrawLegend(float x, float y, std::string y1, std::string y2, std::string y3, std::string y4)
{
  if( !y1.empty() )
  {
    glColor3f(m_y1_color.x, m_y1_color.y, m_y1_color.z);
    glBegin(GL_LINES);
    glVertex3f(x, y, 0.0);
    glVertex3f(x + 20, y, 0.0);
    glEnd();
    g_font->Print(y1.c_str(), m_fontsize, x + 25, y - 2, m_fontcolor.x, m_fontcolor.y, m_fontcolor.z);
  }
  y = y - 20;
  if( !y2.empty() )
  {
    glColor3f(m_y2_color.x, m_y2_color.y, m_y2_color.z);
    glBegin(GL_LINES);
    glVertex3f(x, y, 0.0);
    glVertex3f(x + 20, y, 0.0);
    glEnd();
    g_font->Print(y2.c_str(), m_fontsize, x + 25, y - 2, m_fontcolor.x, m_fontcolor.y, m_fontcolor.z);
  }
  y = y - 20;
  if( !y3.empty() )
  {
    glColor3f(m_y3_color.x, m_y3_color.y, m_y3_color.z);
    glBegin(GL_LINES);
    glVertex3f(x, y, 0.0);
    glVertex3f(x + 20, y, 0.0);
    glEnd();
    g_font->Print(y3.c_str(), m_fontsize, x + 25, y - 2, m_fontcolor.x, m_fontcolor.y, m_fontcolor.z);
  }
  y = y - 20;
  if( !y4.empty() )
  {
    glColor3f(m_y4_color.x, m_y4_color.y, m_y4_color.z);
    glBegin(GL_LINES);
    glVertex3f(x, y, 0.0);
    glVertex3f(x + 20, y, 0.0);
    glEnd();
    g_font->Print(y4.c_str(), m_fontsize, x + 25, y - 2, m_fontcolor.x, m_fontcolor.y, m_fontcolor.z);
  }

}

void tgPlot2D::DrawData(const std::vector<float> &x, const std::vector<float> &y) const
{

  if( x.empty() || y.empty() )
    return;

  unsigned m = std::min(x.size(), y.size());

  glBegin(GL_LINE_STRIP);
  for( unsigned i = 0; i < m; i++ )
  {
    glVertex3f(this->x + (x[i] - this->x_min) * this->x_scale, this->y + (y[i] - this->y_min) * this->y_scale, 0.0f);
  }
  glEnd();
}

void tgPlot2D::Push(float y1)
{
  //if(y1<y_min) axis(x_min, x_max, y1, y_max);
  //if(y1>y_max) axis(x_min, x_max, y_min, y1);

  updateBuffer(y1);

  glColor3f(m_y1_color.x, m_y1_color.y, m_y1_color.z);
  DrawData(m_buffer_x, m_buffer_y_1);
}

void tgPlot2D::Push(float y1, float y2)
{
  //if(y1<y_min) axis(x_min, x_max, y1, y_max);
  //if(y1>y_max) axis(x_min, x_max, y_min, y1);
  //if(y2<y_min) axis(x_min, x_max, y2, y_max);
  //if(y2>y_max) axis(x_min, x_max, y_min, y2);

  updateBuffer(y1, y2);

  glColor3f(m_y1_color.x, m_y1_color.y, m_y1_color.z);
  DrawData(m_buffer_x, m_buffer_y_1);
  glColor3f(m_y2_color.x, m_y2_color.y, m_y2_color.z);
  DrawData(m_buffer_x, m_buffer_y_2);
}

void tgPlot2D::Push(float y1, float y2, float y3)
{
  //if(y1<y_min) axis(x_min, x_max, y1, y_max);
  //if(y1>y_max) axis(x_min, x_max, y_min, y1);
  //if(y2<y_min) axis(x_min, x_max, y2, y_max);
  //if(y2>y_max) axis(x_min, x_max, y_min, y2);
  //if(y3<y_min) axis(x_min, x_max, y3, y_max);
  //if(y3>y_max) axis(x_min, x_max, y_min, y3);

  updateBuffer(y1, y2, y3);

  glColor3f(m_y1_color.x, m_y1_color.y, m_y1_color.z);
  DrawData(m_buffer_x, m_buffer_y_1);
  glColor3f(m_y2_color.x, m_y2_color.y, m_y2_color.z);
  DrawData(m_buffer_x, m_buffer_y_2);
  glColor3f(m_y3_color.x, m_y3_color.y, m_y3_color.z);
  DrawData(m_buffer_x, m_buffer_y_3);
}

void tgPlot2D::Push(float y1, float y2, float y3, float y4)
{
  //if(y1<y_min) axis(x_min, x_max, y1, y_max);
  //if(y1>y_max) axis(x_min, x_max, y_min, y1);
  //if(y2<y_min) axis(x_min, x_max, y2, y_max);
  //if(y2>y_max) axis(x_min, x_max, y_min, y2);
  //if(y3<y_min) axis(x_min, x_max, y3, y_max);
  //if(y3>y_max) axis(x_min, x_max, y_min, y3);
  //if(y4<y_min) axis(x_min, x_max, y4, y_max);
  //if(y4>y_max) axis(x_min, x_max, y_min, y4);

  updateBuffer(y1, y2, y3, y4);

  glColor3f(m_y1_color.x, m_y1_color.y, m_y1_color.z);
  DrawData(m_buffer_x, m_buffer_y_1);
  glColor3f(m_y2_color.x, m_y2_color.y, m_y2_color.z);
  DrawData(m_buffer_x, m_buffer_y_2);
  glColor3f(m_y3_color.x, m_y3_color.y, m_y3_color.z);
  DrawData(m_buffer_x, m_buffer_y_3);
  glColor3f(m_y4_color.x, m_y4_color.y, m_y4_color.z);
  DrawData(m_buffer_x, m_buffer_y_4);
}
