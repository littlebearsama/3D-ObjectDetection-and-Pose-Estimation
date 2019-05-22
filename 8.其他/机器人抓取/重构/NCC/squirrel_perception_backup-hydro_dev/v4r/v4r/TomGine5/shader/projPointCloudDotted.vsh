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
in vec2 position;
in vec2 TextureCoordinates;
out vec4 color;

uniform vec2 f;
uniform vec2 c;
uniform float depthScale;
uniform mat4 mvp;

uniform bool diffuseShading;
uniform vec3 lightPos;
uniform vec3 lightDir;
uniform vec3 diffuseLight;
uniform vec3 diffuseColor;

uniform sampler2D depthTex;
uniform sampler2D rgbTex;

vec3 computeNormal(in vec3 pos)
{
  float dR = -textureOffset(depthTex,TextureCoordinates,ivec2( 1, 0)).x*depthScale;
  float dT = -textureOffset(depthTex,TextureCoordinates,ivec2( 0, 1)).x*depthScale;
  float dL = -textureOffset(depthTex,TextureCoordinates,ivec2(-1, 0)).x*depthScale;
  float dB = -textureOffset(depthTex,TextureCoordinates,ivec2( 0,-1)).x*depthScale;
  vec3 posR=vec3( -(position.x+1-c.x)*dR/f.x, (position.y+0-c.y)*dR/f.y, dR );
  vec3 posT=vec3( -(position.x+0-c.x)*dT/f.x, (position.y+1-c.y)*dT/f.y, dT );
  vec3 posL=vec3( -(position.x-1-c.x)*dL/f.x, (position.y+0-c.y)*dL/f.y, dL );
  vec3 posB=vec3( -(position.x+0-c.x)*dB/f.x, (position.y-1-c.y)*dB/f.y, dB );
  vec3 vR = posR-pos;
  vec3 vT = posT-pos;
  vec3 vL = posL-pos;
  vec3 vB = posB-pos;
  vec3 n0 = normalize(cross(vR,vT));
  vec3 n1 = normalize(cross(vT,vL));
  vec3 n2 = normalize(cross(vL,vB));
  vec3 n3 = normalize(cross(vB,vR));
  return normalize((n0+n1+n2+n3)*0.25);
//  return n0;
}

void main()
{
  float d = -texture(depthTex,TextureCoordinates).x*depthScale;
  vec3 pos=vec3( -(position.x-c.x)*d/f.x, (position.y-c.y)*d/f.y, d );

  if(diffuseShading)
  {
    vec3 n = computeNormal(pos);
//    vec3 s = normalize(lightPos - pos);
    color = vec4(diffuseLight * diffuseColor.rgb * max( dot(lightDir,n), 0.0 ), 1.0);
  }
  else
  {
    color=texture(rgbTex,TextureCoordinates);
    color.g=color.r;
    color.b=color.r;
  }

  gl_Position = mvp*vec4(pos,1);
}
