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

#ifndef TG_LIGHTING
#define TG_LIGHTING

#include "headers.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Light settings, representing a light. See glLight in the OpenGL spec. for more information. */
class tgLight
{
public:
	vec4 ambient;	///< ambient RGBA intensity of light.
	vec4 diffuse;	///< diffuse RGBA intensity of light.
	vec4 specular;	///< specular RGBA intensity of light.
	vec4 position;	///< position of the light in homogeneous coordinates.

	/** @brief Create white light. */
	tgLight();

	/** @brief Enables lighting and applies colors and position of the light.
	 *  @param id	OpenGL light id ranging from 0 to GL_MAX_LIGHTS-1 */
	void Activate(int id=0) const;

	/** @brief Creates light with a specific color intensity RGBA */
	void Color(float r, float g, float b, float a=1.0f);

	/** @brief Create light with random color intensity RGBA. */
	void Random();

	/** @brief Draws the light as a dot or line depending on its 4th value */
	void Draw();
};

} // namespace TomGine

#endif
