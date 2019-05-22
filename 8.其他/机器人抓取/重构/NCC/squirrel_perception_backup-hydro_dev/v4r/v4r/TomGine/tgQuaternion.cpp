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

#include "tgQuaternion.h"
#include <stdio.h>

using namespace TomGine;

tgQuaternion::tgQuaternion(){
	x=0.0;
	y=0.0;
	z=0.0;
	w=1.0;
}

tgQuaternion::tgQuaternion(float x, float y, float z, float w){
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

// normalising a quaternion works similar to a vector. This method will not do anything
// if the quaternion is close enough to being unit-length. define TOLERANCE as something
// small like 0.00001f to get accurate results
void tgQuaternion::normalise(){
	// Don't normalize if we don't have to
	float mag2 = w * w + x * x + y * y + z * z;
	if (fabs(mag2 - 1.0f) > epsilon) {
		float mag = sqrt(mag2);
		w /= mag;
		x /= mag;
		y /= mag;
		z /= mag;
	}
}

// We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a vector
// The conjugate of a quaternion is the same as the inverse, as long as the quaternion is unit-length
tgQuaternion tgQuaternion::getConjugate() const{
	return tgQuaternion(-x, -y, -z, w);
}

bool tgQuaternion::operator==(const tgQuaternion &q) const{
	return (fabs(x-q.x)<epsilon &&
			fabs(y-q.y)<epsilon &&
			fabs(z-q.z)<epsilon &&
			fabs(w-q.w)<epsilon);
}

// Adding
tgQuaternion tgQuaternion::operator+ (const tgQuaternion &q2) const{
	tgQuaternion rq;
	rq.x = x+q2.x;
	rq.y = y+q2.y;
	rq.z = z+q2.z;
	rq.w = w+q2.w;
	
	return rq;
}

// Subtracting
tgQuaternion tgQuaternion::operator- (const tgQuaternion &q2) const{
	tgQuaternion rq;
	rq.x = x-q2.x;
	rq.y = y-q2.y;
	rq.z = z-q2.z;
	rq.w = w-q2.w;
	return rq;
}


// Multiplying q1 with q2 applies the rotation q2 to q1
tgQuaternion tgQuaternion::operator* (const tgQuaternion &rq){
	// the constructor takes its arguments as (x, y, z, w)
	return tgQuaternion(w * rq.x + x * rq.w + y * rq.z - z * rq.y,
	                  w * rq.y + y * rq.w + z * rq.x - x * rq.z,
	                  w * rq.z + z * rq.w + x * rq.y - y * rq.x,
	                  w * rq.w - x * rq.x - y * rq.y - z * rq.z);
}

tgQuaternion tgQuaternion::operator* (const float f){
	return tgQuaternion(x*f, y*f, z*f, w*f);
}


// Multiplying a quaternion q with a vector v applies the q-rotation to v
vec3 tgQuaternion::operator* (vec3 v){
	v.normalize();
 
	tgQuaternion vecQuat, resQuat;
	vecQuat.x = v.x;
	vecQuat.y = v.y;
	vecQuat.z = v.z;
	vecQuat.w = 0.0f;
 
	resQuat = vecQuat * getConjugate();
	resQuat = *this * resQuat;
 
	return (vec3(resQuat.x, resQuat.y, resQuat.z));
}

// Convert from Axis Angle
void tgQuaternion::fromAxis(const vec3 &v, float angle){
	float sinAngle;
	angle *= 0.5f;
	vec3 vn(v);
	vn.normalize();
 
	sinAngle = sin(angle);
 
	x = (vn.x * sinAngle);
	y = (vn.y * sinAngle);
	z = (vn.z * sinAngle);
	w = cos(angle);
}

// Convert from Euler Angles
void tgQuaternion::fromEuler(float roll, float pitch, float yaw){
	// Basically we create 3 tgQuaternions, one for pitch, one for yaw, one for roll
	// and multiply those together.
	// the calculation below does the same, just shorter
 
	float p = pitch / 2.0f;
	float y = yaw / 2.0f;
	float r = roll / 2.0f;
 
	float sinp = sin(p);
	float siny = sin(y);
	float sinr = sin(r);
	float cosp = cos(p);
	float cosy = cos(y);
	float cosr = cos(r);
 
	this->x = sinr * cosp * cosy - cosr * sinp * siny;
	this->y = cosr * sinp * cosy + sinr * cosp * siny;
	this->z = cosr * cosp * siny - sinr * sinp * cosy;
	this->w = cosr * cosp * cosy + sinr * sinp * siny;
 
	normalise();
}

// Convert from Matrix 4x4
void tgQuaternion::fromMatrix(mat4 m){
	w = sqrt(1.0f + m[0] + m[5] + m[10]) / 2.0f;
	float w4 = (4.0f * w);
	x = (m[9] - m[6]) / w4 ;
	y = (m[2] - m[8]) / w4 ;
	z = (m[4] - m[1]) / w4 ;
	
	normalise();
}

// Convert from Matrix 3x3
void tgQuaternion::fromMatrix(mat3 m){
	w = sqrt(1.0f + m[0] + m[4] + m[8]) / 2.0f;
	float w4 = (4.0f * w);
	x = (m[7] - m[5]) / w4 ;
	y = (m[2] - m[6]) / w4 ;
	z = (m[3] - m[1]) / w4 ;
	
	normalise();
}

// Convert to Matrix 4x4
mat4 tgQuaternion::getMatrix4() const{
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;
	float xy = x * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;
	float wy = w * y;
	float wz = w * z;
 
	mat4 rot;
	rot[0]=1.0f - 2.0f * (y2 + z2);	rot[1]=2.0f * (xy - wz);		rot[2]=2.0f * (xz + wy);			rot[3]=0.0f;
	rot[4]=2.0f * (xy + wz); 		rot[5]=1.0f - 2.0f * (x2 + z2);	rot[6]=2.0f * (yz - wx);			rot[7]=0.0f;
	rot[8]=2.0f * (xz - wy);		rot[9]=2.0f * (yz + wx);		rot[10]=1.0f - 2.0f * (x2 + y2);	rot[11]=0.0f;
	rot[12]=0.0f;					rot[13]=0.0f;					rot[14]=0.0f;						rot[15]=1.0f;
	
	return rot;
}

// Convert to Matrix 3x3
mat3 tgQuaternion::getMatrix3() const{
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;
	float xy = x * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;
	float wy = w * y;
	float wz = w * z;
	
	mat3 rot;
	rot[0]=1.0f - 2.0f * (y2 + z2);	rot[1]=2.0f * (xy - wz);		rot[2]=2.0f * (xz + wy);
	rot[3]=2.0f * (xy + wz); 		rot[4]=1.0f - 2.0f * (x2 + z2);	rot[5]=2.0f * (yz - wx);
	rot[6]=2.0f * (xz - wy);		rot[7]=2.0f * (yz + wx);		rot[8]=1.0f - 2.0f * (x2 + y2);
	
	return rot;
}


// Convert to Axis/Angles
void tgQuaternion::getAxisAngle(vec3& axis, float& angle) const{
	float scale = sqrt(x * x + y * y + z * z);
	axis.x = x / scale;
	axis.y = y / scale;
	axis.z = z / scale;
	angle = acos(w) * 2.0f;
	if(angle < 0.0 || angle > (2.0*M_PI))
		angle = 0.0;
}

void tgQuaternion::print() const{
  printf("x: %f y: %f z: %f w: %f\n", x, y, z, w);
}

void tgQuaternion::printAxisAngle() const{
  vec3 axis;
  float angle;
  getAxisAngle(axis, angle);
  printf("%f, %f, %f - %f\n", axis.x, axis.y, axis.z, angle);
}
