/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2014, Simon Schreiberhuber
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
 * @author simon.schreiberhuber
 *
 */

#version 430

in vec3 VertexPosition;
in vec3 VertexNormal;
in vec4 ColorDiffuse;

out vec3 LightIntensity;

uniform vec4 LightPosition;
uniform vec3 LightDiffuse;

uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 MVP;

void main()
{
  vec3 tnorm = normalize(NormalMatrix * VertexNormal);
  vec4 eyeCoords = ModelViewMatrix * vec4(VertexPosition,1.0);
  vec3 s = normalize(vec3(LightPosition - eyeCoords));

  // diffuse shading
  LightIntensity = LightDiffuse * ColorDiffuse.rgb * max( dot(s,tnorm), 0.0 );

  gl_Position = MVP*vec4(VertexPosition,1.0);
}
