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
in vec3 homography1;
in vec3 homography2;
in vec3 homography3;

uniform int sqrt_maxPatchCount;
uniform int pyrLvl;
uniform int patchSize;
uniform sampler2D intensityTexture;

out vec2 texCoord;
out float ind;

/*
 * Generates the texture coordinates for the homographic read operation:
 */
void main(){

    //Unluckily glsl does not allow for matrix parameters:
    mat3 homography;
    homography[0]=homography1;
    homography[1]=homography2;
    homography[2]=homography3;

    ivec2 texRes=textureSize(intensityTexture,0);
    float scale=pow(2.0,pyrLvl);
    vec3 pix = transpose(homography)*vec3(position*patchSize*scale,1);
    vec2 p= pix.xy/pix.z;

    //Good for us: the homography only needs to be evaluated on the edges of the patch
    texCoord.x=p.x/float(texRes.x);
    texCoord.y=p.y/float(texRes.y);

    //calculate at wich position the patch should be rendered to:
    float col = float(mod(float(index),sqrt_maxPatchCount))/float(sqrt_maxPatchCount)*2.0;
    float row = float(floor(float(index)/sqrt_maxPatchCount))/float(sqrt_maxPatchCount)*2.0;
    gl_Position = vec4(position*2.0/float(sqrt_maxPatchCount)-vec2(1 - col,1 - row),0.0,1.0);



    ind=index;
}
