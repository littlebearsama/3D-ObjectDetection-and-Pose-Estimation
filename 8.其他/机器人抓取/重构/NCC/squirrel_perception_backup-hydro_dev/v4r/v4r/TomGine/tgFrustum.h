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

#ifndef TG_FRUSTUM
#define TG_FRUSTUM

#include "headers.h"

#include "tgMathlib.h"

namespace TomGine{

/** @brief View frustum, defined by 6 planes. (near, far, left, right, bottom, top) */
class tgFrustum{
    
public:
	float frustum[6][4];	///< The plane parameters
	mat4 m_intrinsic;		///< intrinsic camera matrix
	mat4 m_extrinsic;		///< extrinsic camera matrix
	vec3 m_color;

	void ExtractFrustum (const mat4 &i, const mat4 &e);

	/** @brief Extracts the view frustum from the currently set projection- and modelview-matrix. */
	void ExtractFrustum();

	friend class tgCamera;

public:

	/** @brief Check whether a point lies in the frustum.
	 *  @return true	Point lies in the frustum.
	 *  @return false	Point lies outside the frustum.
	 *  @param x,y,z	The point to check. */
	bool PointInFrustum( float x, float y, float z );

	/** @brief Check whether a shpere lies completely in the frustum.
	 *  @return true	Sphere lies completely in the frustum.
	 *  @return false	Point lies completely outside the frustum.
	 *  @param x,y,z	The center of the sphere.
	 *  @param radius	The readius of the sphere. */
	bool SphereInFrustum( float x, float y, float z, float radius );

	/** @brief Draws the view frustum using GL_LINES. */
	void DrawFrustum();

};

} // namespace TomGine

#endif
