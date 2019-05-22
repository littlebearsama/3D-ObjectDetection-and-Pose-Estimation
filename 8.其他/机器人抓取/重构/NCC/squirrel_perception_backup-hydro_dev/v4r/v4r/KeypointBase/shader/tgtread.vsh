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
in vec3 tgtPos;

uniform int sqrt_maxPatchCount;
uniform int pyrLvl;
uniform int patchSize;
uniform sampler2D intensityTexture;


out vec2 texCoord;

/*
 * Reads the patch out of the srcImage and writes it to a texture.
 */
void main(){


    ivec2 texRes=textureSize(intensityTexture,0);
    float scale = pow(2.0,pyrLvl);
    vec2 p = position*patchSize - vec2(patchSize/2);
    p=p*scale;

    texCoord=p+tgtPos.xy;

    texCoord.x=texCoord.x/float(texRes.x);
    texCoord.y=texCoord.y/float(texRes.y);

    //calculate the position of the render target:
    float col = float(mod(float(index),sqrt_maxPatchCount))/float(sqrt_maxPatchCount)*2.0;
    float row = float(floor(float(index)/sqrt_maxPatchCount))/float(sqrt_maxPatchCount)*2.0;
    gl_Position = vec4(position*2.0/float(sqrt_maxPatchCount)-vec2(1 - col,1 - row),0.0,1.0);
}
