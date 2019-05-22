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
uniform int sqrt_maxPatchCount;
uniform int patchCount;
uniform sampler2D srcTexture;

out vec2 tc;
out vec2 tl;
out vec2 t;
out vec2 tr;
out vec2 l;
out vec2 r;
out vec2 bl;
out vec2 b;
out vec2 br;

/*
 * Generates a quad around all patches so that gxyerr can be calcualted around them.
 */
void main(){

    //calculate how many rows of this image are filled with patches
    float row = (float(floor(float(patchCount)/float(sqrt_maxPatchCount))) + 1)/float(sqrt_maxPatchCount) ;
    vec2 p=vec2(position.x,position.y*row);
    gl_Position = vec4(p*2.0-vec2(1),0.0,1.0);

    tc=p;

    ivec2 texRes=textureSize(srcTexture,0);
    float pw=1.0/float(texRes.x);//pixel width

    //calculate the texture coordinates for the following soebel operation:
    tl= vec2(tc.x-pw,tc.y-pw);
    t = vec2(tc.x,tc.y-pw);
    tr = vec2(tc.x+pw,tc.y-pw);
    l = vec2(tc.x-pw,tc.y);
    r = vec2(tc.x+pw,tc.y);
    bl= vec2(tc.x-pw,tc.y+pw);
    b = vec2(tc.x,tc.y+pw);
    br = vec2(tc.x+pw,tc.y+pw);

    //gl_Position=vec4(position.x/2,position.y/2,0,1);
    //gl_Position=vec4(0);
    //texCoord=gl_Position.xy/2.0+vec2(0.5,0.5);
}
