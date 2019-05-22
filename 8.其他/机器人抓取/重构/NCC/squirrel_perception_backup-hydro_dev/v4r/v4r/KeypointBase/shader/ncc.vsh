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

in vec2 position;
in int index;

uniform int sqrt_maxPatchCount;
uniform sampler2D tgtTexture;
uniform sampler2D srcTexture;
uniform int lod;

out vec2 texCoord;
out vec3 texColor;
out float m1;
out float m2;

/*
 * calculate the Normalized Cross Correlation
 */
void main(){


    ivec2 texRes=textureSize(tgtTexture,0);
    int patchSize=texRes.x/sqrt_maxPatchCount;

    vec2 p = position*patchSize;


    //get the position of the current patch
    float col = float(mod(float(index),sqrt_maxPatchCount))/float(sqrt_maxPatchCount);
    float row = float(floor(float(index)/sqrt_maxPatchCount))/float(sqrt_maxPatchCount);

    gl_Position = vec4(position*2.0/float(sqrt_maxPatchCount)-vec2(1 - col*2,1 - row*2),0.0,1.0);
    texCoord=gl_Position.xy/2.0 + vec2(0.5,0.5);

    vec2 tc = vec2(col,row)+vec2(0.5/float(sqrt_maxPatchCount));

    //we read out the average value of this patch and put it to the uniform:
    m1 = textureLod(tgtTexture,tc,lod).x;
    m2 = textureLod(srcTexture,tc,lod).x;
}
