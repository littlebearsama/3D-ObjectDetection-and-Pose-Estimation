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

#include "tgMathlib.h"
#include <stdio.h>

using namespace TomGine;

void vec3::rotate(float alpha, vec3 r)
{
	vec3 v,s,t,n;
	mat3 M, Mt, Rx, X;

	if(alpha != 0.0f){

		r.normalize();
		s.cross(r,vec3(1,0,0));

		if(s.length() < 0.001f)
			s.cross(r, vec3(0,1,0));

		s.normalize();
		t.cross(r,s);

		Mt = mat3(r,s,t);
		M = mat3(Mt);
		M = M.transpose();

		Rx = mat3(	1.0f,	0.0f,	0.0f,
					0.0f,	cosf(alpha),	-sinf(alpha),
					0.0f,	sinf(alpha),	cosf(alpha));

		X = Mt*Rx*M;

		v = vec3(x,y,z);
		n = X*v;

		x = n.x;
		y = n.y;
		z = n.z;
	}
}
