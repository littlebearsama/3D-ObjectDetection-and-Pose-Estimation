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

#version 130

in vec2 texCoord;
in float m1;//average values read by the vertex shader
in float m2;
in vec3 texColor;

uniform sampler2D tgtTexture;
uniform sampler2D srcTexture;

out vec4 fragColor;

/*
 * calculate the Normalized Cross Correlation
 */
void main(){

    float s1 = textureLod(tgtTexture,texCoord,0).x-m1;
    float s2 = textureLod(srcTexture,texCoord,0).x-m2;

    float dist = s1*s2;
    s1=s1*s1;
    s2=s2*s2;
    fragColor = vec4(s1,s2,dist,0);//the rest of the calculation will be done after mipmapping by the cpu
}
