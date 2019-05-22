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

#ifndef TG_COLLISSION_H
#define TG_COLLISSION_H

#include <vector>
#include "tgMathlib.h"
#include "tgModel.h"
#include "tgRenderModel.h"

namespace TomGine {

/** @brief Intersection and collission tests of triangles, rays, models, ... */
class tgCollission
{
private:
  static bool valueInRange(int value, int min, int max)
  {
    return (value >= min) && (value <= max);
  }

public:
  /** @brief Get normal vector of a triangle.
   *  @param v1,v2,v3 vertices of the triangle. */
  static vec3 GetNormal(const vec3& v1, const vec3& v2, const vec3& v3);

  /** @brief Intersetion tests of two rectangles in 2D
   *  @param ll1,ll2		lower left corner of rectangle 1 and 2
   *  @param ur2,ur2		upper right corner of rectangle 1 and 2	 */
  static bool IntersectRectangles2D(const tgRect2D& A, const tgRect2D& B);

  /** @brief Intersection test between ray and triangle.
   *  @return bool	true if ray and triangle intersect, false if no intersection exists.
   *  @return &p		point of intersection.
   *  @return &n		normal at point of intersetion.
   *  @return &z		distance from start point of ray to intersection point.
   *  @param ray		ray for intersection.
   *  @param t1,t2,t3 vertices of triangle for intersection	. */
  static bool IntersectRayTriangle(vec3& p, vec3& n, double& z, const tgRay& ray, const vec3& t1, const vec3& t2, const vec3& t3);

  /** @brief Intersection test between ray and sphere.
   *  @return bool  true if ray and sphere intersect, false if no intersection exists.
   *  @return &t    intersection point, distance from origin along ray direction
   *  @param ray    ray for intersection.
   *  @param sphere sphere for intersection	 */
  static bool IntersectRaySphere(const tgRay &ray, const BoundingSphere &sphere, float &t);

  /** @brief Intersection test between ray and model.
   *  @return bool	true if ray and model intersect, false if no intersection exists.
   *  @return &pl		point-list of intersections.
   *  @return &nl		normal-list of points of intersections.
   *  @return &zl		distance-list from start point of ray to intersection points.
   *  @param ray		ray for intersection.
   *  @param model	model for intersection.	 */
  static bool IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray,
      const tgModel& model);

  /** @brief Intersection test between ray and model.
   *  @return bool  true if ray and model intersect, false if no intersection exists.
   *  @return &pl   point-list of intersections.
   *  @return &nl   normal-list of points of intersections.
   *  @return &zl   distance-list from start point of ray to intersection points.
   *  @param ray    ray for intersection.
   *  @param model  model for intersection.  */
  static bool IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray,
      const tgRenderModel& model);

  /** @brief not implemented */
  static bool IntersectModels(const tgModel &m1, const tgModel &m2, const tgPose &p1, const tgPose& p2);
  /** @brief not implemented */
  static bool IntersectModels(const tgRenderModel &m1, const tgRenderModel &m2);

  /** @brief Tests if point 'p1' lies on the same side as 'p2'.
   *  @param p1,p2	Points to test if they lie on the same side.
   *  @param a,b		Line to test points against.
   *  @return bool	true if points 'p1','p2' are on the same side with respect to the line 'a','b'	 */
  static bool PointOnSameSide(const vec3& p1, const vec3& p2, const vec3& a, const vec3& b);

  /** @brief Tests if a point lies in or outside a triangle.
   *  @param p		Point to test
   *  @param t1,t2,t3 Triangle given by vertices
   *  @return bool	true if point lies in triangle, false if outside. 	 */
  static bool PointInTriangle(const vec3& p, const vec3& t1, const vec3& t2, const vec3& t3);

};

}

#endif /* TG_COLLISSION_H */
