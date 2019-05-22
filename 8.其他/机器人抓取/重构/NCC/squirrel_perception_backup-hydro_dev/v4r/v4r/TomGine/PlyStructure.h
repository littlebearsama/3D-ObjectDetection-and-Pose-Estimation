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

#ifndef TOMGINE_PLY_STRUCTURE
#define TOMGINE_PLY_STRUCTURE

namespace TomGine{
  
struct PlyVertex {
	float x,y,z;                // spatial position
	float nx, ny, nz;           // normal vector
	float s, t;
  unsigned char r, g, b;      // color
  PlyVertex() :
    x(0.0f),y(0.0f),z(0.0f),
    nx(0.0f),ny(0.0f),nz(0.0f),
    s(0.0f),t(0.0f),
    r(255),g(255),b(255)
  {}

};

struct PlyFace {
	unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
	unsigned int* v;            // pointer to memory holding the vertex-index list
  PlyFace() : nverts(0), v(NULL) {}
};

struct PlyEdge {
	unsigned short start;       // start vertex index
	unsigned short end;         // end vertex index
  PlyEdge(): start(0), end(0){}
};

}

#endif
