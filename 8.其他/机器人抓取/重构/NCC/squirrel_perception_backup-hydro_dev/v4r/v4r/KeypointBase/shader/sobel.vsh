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

#version 410

in vec2 position;



//uniform int patchSize;
uniform sampler2D intensityTexture;


out vec2 tc;
out vec2 tl;
out vec2 t;
out vec2 tr;
out vec2 l;
out vec2 r;
out vec2 bl;
out vec2 b;
out vec2 br;

void main(){


    ivec2 texRes=textureSize(intensityTexture,0);
    float pwh=1.0/float(texRes.x);//pixel width horizontal
    float pwv=1.0/float(texRes.y);
    tc=position;

    tl= vec2(tc.x-pwh,tc.y-pwv);
    t = vec2(tc.x,tc.y-pwv);
    tr = vec2(tc.x+pwh,tc.y-pwv);
    l = vec2(tc.x-pwh,tc.y);
    r = vec2(tc.x+pwh,tc.y);
    bl= vec2(tc.x-pwh,tc.y+pwv);
    b = vec2(tc.x,tc.y+pwv);
    br = vec2(tc.x+pwh,tc.y+pwv);
    gl_Position=vec4(position.x*2-1,position.y*2-1,0,1);
    //gl_Position=vec4(0);
    //texCoord=gl_Position.xy/2.0+vec2(0.5,0.5);
}
