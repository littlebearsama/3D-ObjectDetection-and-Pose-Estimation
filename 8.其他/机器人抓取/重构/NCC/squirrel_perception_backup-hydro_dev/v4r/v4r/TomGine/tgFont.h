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

#ifndef _TG_FONT_H_
#define _TG_FONT_H_

#include "tgSingleton.h"
#include "tgMathlib.h"
#include <string>
#include "headers.h"

#ifdef USE_FTGL_FONT
#include <FTGL/ftgl.h>
#endif

namespace TomGine {

#define g_font tgFont::GetInstance()

/** @brief Contains information for printing a label in 2d */
struct tgLabel2D
{
  tgLabel2D(std::string t, int s = 20, int x = 0, int y = 0, float r = 1.0f, float g = 1.0f, float b = 1.0f) :
    text(t), size(s), x(x), y(y), rgba(r, g, b, 1.0f)
  {
  }
  std::string text; ///< Character string of the label
  int size; ///< Font size of the label in points
  int x; ///< x-position of the label in pixel
  int y; ///< y-position of the label in pixel
  vec4 rgba; ///< color of the label
};
/** @brief Contains information for printing a label in 3d */
struct tgLabel3D
{
  std::string text; ///< Character string of the label
  int size; ///< Font size of the label in points
  vec3 pos; ///< Position of the label in 3D-space
  vec4 rgba; ///< color of the label
};

/** @brief Drawing fonts with OpenGL using FTGL. */
class tgFont
{
public:
  friend class tgSingleton<tgFont> ;
  static tgFont* GetInstance()
  {
    return tgSingleton<tgFont>::GetInstance();
  }

  enum Type
  {
    BITMAP_FONT = 0, PIXMAP_FONT
  };

private:
#ifdef USE_FTGL_FONT
  FTFont* m_font;
#endif
  GLuint m_font_base;
protected:
  /** @brief Create the FTGL font. */
  tgFont();
  /** @brief Destroy the FTGL font. */
  ~tgFont();

public:

  void SetGLXFontBase(GLuint base){ m_font_base = base; }

  void Set(tgFont::Type type);

  /** @brief Print a text in an OpenGL window.
   *  @param text		The text to print as character array.
   *  @param size		The size of the font in points.
   *  @param x,y		The position of the font in image space.
   *  @param r,g,b	The color of the font.
   *  @param a		Transparency of the font.	 */
  void
  Print(const char* text, int size, int x, int y, float r = 1.0f, float g = 1.0f, float b = 1.0f, float a = 1.0f) const;

  /** @brief Print a label in an OpenGL window.
   *  For printing tgLabel3D label:  <BR> <BR>
   * <CODE>
   *  mat4 modelview, projection, modelviewprojection;  <BR>
   *	vec4 viewport, texCoords;  <BR>
   *	float x, y;  <BR> <BR>
   *	glGetFloatv(GL_MODELVIEW_MATRIX, modelview); <BR>
   *	glGetFloatv(GL_PROJECTION_MATRIX, projection); <BR>
   *	glGetFloatv(GL_VIEWPORT, viewport); <BR>
   *	modelviewprojection = projection * modelview; <BR>
   *	texCoords = modelviewprojection * vec4(label.pos.x, label.pos.y, label.pos.z, 1.0); <BR>
   *	x = (texCoords.x / texCoords.w + 1.0f) * 0.5f; <BR>
   *	y = (texCoords.y / texCoords.w + 1.0f) * 0.5f; <BR> <BR>
   *	g_font->Print(label.text.c_str(), label.size, int(viewport.z * x), int(viewport.w * y)); <BR>
   *	</CODE>
   */
  void Print(const tgLabel2D &label) const;

};

}

#endif
