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

#ifndef TG_MATERIAL_MODEL
#define TG_MATERIAL_MODEL

#include "tgMathlib.h"

namespace TomGine {

/** @brief Specify material parameters for the OpenGL lighting model. */
class tgMaterial
{
public:
  vec4 ambient; ///< ambient RGBA reflectance of the material
  vec4 diffuse; ///< diffuse RGBA reflectance of the material
  vec4 specular; ///< specular RGBA reflectance of the material
  vec4 color; ///< RGBA color (used directly if lighting disabled)
  vec4 emission;
  float shininess; ///< RGBA specular exponent of the material

  /** @brief Create grey material. */
  tgMaterial();

  /** @brief Enables lighting and applies material to OpenGL. */
  void Activate() const;
  /** @brief Disables blending. */
  void Deactivate() const;
  /** @brief Applies material to OpenGL. */
  void Apply() const;

  /** @brief Create material with a specific color reflectance RGBA */
  void Color(float r, float g, float b, float a = 1.0f, float amb = 0.7f, float diff = 1.0f, float spec = 0.2f,
      float emis = 0.0f, float shiny = 10.0f);

  void Color(float Ra, float Ga, float Ba, float Aa,
             float Rd, float Gd, float Bd, float Ad,
             float Rs, float Gs, float Bs, float As,
             float shiny);

  /** @brief Create material with random color reflectance RGBA. */
  void Random(float brightness = 1.0f);

  void Exposure(float val);
};

} // namespace TomGine

#endif
