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
 
#ifndef TG_POSE
#define TG_POSE

#include "tgMathlib.h"
#include "tgQuaternion.h"

namespace TomGine{

/** @brief Represents a pose in 3D space with position and orientation.
 *  See Special Euclidean Space SE(3).
 *  Orientation is implemented using quaternions. */
class tgPose{
public:
	vec3 t;				///< Position of the pose. (or translation)
	tgQuaternion q;		///< Orientation of the pose. (or rotation)
	
	/** @brief	Compares two poses for equality. */
	bool operator==(const tgPose &p) const;
	/** @brief	Transform pose p into this coordinate frame. */
	tgPose operator*(const tgPose& p) const;
	/** @brief	Transform vector t into this coordinate frame. */
	vec3 operator*(const vec3& t) const;
	/** @brief	Add poses. */
	tgPose operator+(const tgPose &p) const;
	/** @brief	Subtract poses. */
	tgPose operator-(const tgPose &p) const;
	/** @brief	Calculate the transpose (inverse for similarity poses). */
	tgPose Transpose() const;
	
	/** @brief	Prints the components of the position and orientation to the console. */
	void Print() const;

	/** @brief	Pushes the pose as transformation matrix into the OpenGL matrix stack. */
	void Activate() const;	
	/** @brief	Pops (removes) the pose from the OpenGL matrix stack. */
	void Deactivate() const;
	/**	@brief Draws a simple coordinate frame at this pose */
	void DrawCoordinates(float linelength = 1.0f, float linewidth = 1.0f) const;

  mat4 GetMat4() const;
	
	/** @brief	Set the pose as rotation r and translation p. */
  void SetPose(const mat3 &r, const vec3 &p);
  void SetPose(const mat4 &p);
	/** @brief	Gets the pose as rotation r and translation p. */
	void GetPose(mat3 &r, vec3 &p) const;
	/** @brief      Gets the rotation matrix of the pose. */
	mat3 GetRotation() const;
	/** @brief      Gets the pose as homogenious transformation. */
	mat4 GetPose() const;

	/** @brief	Rotate the pose using Euler angles.
	 *  @param	x,y,z	Rotation about x,y,z axis respectively. */
  void Rotate(const float &x, const float &y, const float &z);
	/** @brief	Rotate pose through length(r) in radians about axis given by r. */
  void RotateAxis(const vec3 &r);
	/** @brief	Rotate the pose using Euler angles.
	 *  @param	r	Rotation about x,y,z axis respectively. */
  void RotateEuler(const vec3 &r);
	/** @brief	Translation in x,y,z direction. */
  void Translate(const float &x, const float &y, const float &z);
	/** @brief	Translation along the vector t by the length of t. */
  void Translate(const vec3 &v);
};

} // namespace TomGine

#endif
