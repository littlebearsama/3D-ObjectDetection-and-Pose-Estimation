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

#include "tgErrorMetric.h"
#include "tgCollission.h"

using namespace TomGine;

vec3 tgErrorMetric::GetRandPointInTriangle(const vec3& v0, const vec3& v1, const vec3& v2, unsigned trials) const
{
	vec3 e1 = v1 - v0;
	vec3 e2 = v2 - v0;
	vec3 p;
	
	unsigned i=0;
// 	float x = float(rand()) / RAND_MAX;
// 	float y = float(rand()) / RAND_MAX;
// 	p = v0 + e1*x + e2*y;
	do{
		float x = float(rand()) / RAND_MAX;
		float y = float(rand()) / RAND_MAX;
		p = v0 + e1*x + e2*y;
		i++;
	}while(!tgCollission::PointInTriangle(p,v0,v1,v2) && i<trials);
	
	if(i==trials){
		printf("[tgErrorMetric::GetRandPointInTriangle] Warning: Number of trials exceeded, no point in triangle found.\n");
		p = vec3();
	}
// 	printf("[tgErrorMetric::GetRandPointInTriangle] Trials: %d\n", i);
	return p;
}

tgErrorMetric::tgErrorMetric(tgModel model, unsigned num_points)
{
// 	double dRandMax = 1.0f/RAND_MAX;
// 	
// 	printf("%d %e\n", RAND_MAX, dRandMax);
	
	// calculate area of model surface
	double Area = 0.0;
	double dArea = 0.0;
	for(unsigned f=0; f<model.m_faces.size(); f++){
		vec3 e1 = model.m_vertices[model.m_faces[f].v[1]].pos - model.m_vertices[model.m_faces[f].v[0]].pos;
		vec3 e2 = model.m_vertices[model.m_faces[f].v[2]].pos - model.m_vertices[model.m_faces[f].v[0]].pos;
		vec3 c;
		c.cross(e1,e2);
		Area += c.length() * 0.5;
	}
	dArea = 1.0/Area;
	
	// for each face uniformly distribute random points over surface
	unsigned points_used = 0;
	for(unsigned f=0; f<model.m_faces.size(); f++){
		// calculate area of face
		vec3 v0 = model.m_vertices[model.m_faces[f].v[0]].pos;
		vec3 v1 = model.m_vertices[model.m_faces[f].v[1]].pos;
		vec3 v2 = model.m_vertices[model.m_faces[f].v[2]].pos;
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		vec3 c;
		vec3 p;
		c.cross(e1,e2);
		double area = c.length() * 0.5;
		
		// calculate number of points with respect to area of the face
		unsigned n = 0;
		if(f==model.m_faces.size()-1)
			n = num_points - points_used;  // use up remaining points
		else{
			n = (unsigned)round(area * dArea * num_points);
			points_used += n;
		}
		
		// distribute points randomly over face (distinguish triangles and quads)
		if(model.m_faces[f].v.size() == 4){
			vec3 v3 = model.m_vertices[model.m_faces[f].v[3]].pos;
			unsigned m = n/2;
			for(unsigned i=0; i<m; i++){
				p = GetRandPointInTriangle(v0, v1, v3);
				pointlist.push_back(p);
			}
			for(unsigned i=0; i<(n-m); i++){
				p = GetRandPointInTriangle(v1, v2, v3);
				pointlist.push_back(p);
			}
		}else if(model.m_faces[f].v.size() == 3){
			for(unsigned i=0; i<n; i++){
				p = GetRandPointInTriangle(v0, v1, v2);
				pointlist.push_back(p);
			}
		}else{
			printf("[tgErrorMetric::tgErrorMetric] Warning face size not supported (only quads or triangles)\n");
		}
	}
}


vec3 tgErrorMetric::GetMean(const TomGine::tgPose &p1, const TomGine::tgPose &p2)
{
	double err_x = 0.0;
	double err_y = 0.0;
	double err_z = 0.0;
	double dN = 1.0 / pointlist.size();
	for(unsigned i=0; i<pointlist.size(); i++){
		mat3 R1, R2;
		vec3 t1, t2;
		
		p1.GetPose(R1,t1);
		p2.GetPose(R2,t2);
		
		vec3 verr = (R1 * pointlist[i] + t1) - (R2 * pointlist[i] + t2);
		
		err_x += double(fabs(verr.x)) * dN;
		err_y += double(fabs(verr.y)) * dN;
		err_z += double(fabs(verr.z)) * dN;
	}
	vec3 err = vec3(err_x, err_y, err_z);
	return err;
}

