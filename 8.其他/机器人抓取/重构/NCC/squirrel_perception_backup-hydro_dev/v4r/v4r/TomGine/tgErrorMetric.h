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

#ifndef ERROR_METRIC_H
#define ERROR_METRIC_H

#include <vector>

#include "tgModel.h"
#include "tgPose.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Measures the distance between two poses of an object.
 *
 *	Randomly distribute points on the surface of the model and measure the distance between corresponding points.
 *	*/
class tgErrorMetric
{
private:
	
	/** @brief Generate a random point within a triangle */
	vec3 GetRandPointInTriangle(const vec3& t1, const vec3& t2, const vec3& t3, unsigned trials=100) const;
	
public:
	std::vector<vec3> pointlist;	///< points randomly distributed on the surface of the model.
	
	/** @brief Initialise the error metric.
	 *  @param model	The geometry of the surface on which the points are distributed.
	 *  @param num_points	Number of points generated.	 */
	tgErrorMetric(TomGine::tgModel model, unsigned num_points=10000);
	
	/** @brief Get mean error between poses.
	 * 	@param p1,p2	Poses to compare.
	 * 	@return vec3	Error in each direction in object space. (absolute error = vec3::length()) */
	vec3 GetMean(const TomGine::tgPose &p1, const TomGine::tgPose &p2);
	
	/** @brief Copy pointlist. */
	void GetPoints(std::vector<vec3> &pl){ pl = pointlist; }
	
	/** @brief Get the number of points distributed on the object surface. */
	inline unsigned GetNumPoints(){ return pointlist.size(); }
	
};

} // namespace TomGine

#endif
