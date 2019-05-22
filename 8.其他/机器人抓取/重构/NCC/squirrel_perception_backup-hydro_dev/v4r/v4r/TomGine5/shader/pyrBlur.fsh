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

subroutine vec4 blur();
subroutine uniform blur blurSelect;
in vec2 texCoord;
uniform sampler2D tex;
uniform float lod;
uniform float scale;
out vec4 fragColor;


subroutine (blur)
vec4 blurh(){
    vec2 texRes=textureSize(tex,0);

    float pwh=1.0/texRes.x*scale;
    float pwv=1.0/texRes.y*scale;

    vec2 tc=texCoord;
    vec2 tc_2=vec2(tc.x-2*pwh,tc.y);
    vec2 tc_1=vec2(tc.x-1*pwh,tc.y);
    vec2 tc1=vec2(tc.x+1*pwh,tc.y);
    vec2 tc2=vec2(tc.x+2*pwh,tc.y);

    vec4 c=textureLod(tex,tc,0)*6.0/21.0;
    c+=textureLod(tex,tc_1,0)*4.0/21.0;
    c+=textureLod(tex,tc_2,0)*1.0/21.0;
    c+=textureLod(tex,tc1,0)*4.0/21.0;
    c+=textureLod(tex,tc2,0)*6.0/21.0;
    return c;
}

subroutine (blur)
vec4 blurv(){


    vec2 texRes=textureSize(tex,0);
    float pwh=1.0/texRes.x*1.0;
    float pwv=1.0/texRes.y*1.0;

    vec2 tc=texCoord;
    vec2 tc_2=vec2(tc.x,tc.y-2*pwv);
    vec2 tc_1=vec2(tc.x,tc.y-1*pwv);
    vec2 tc1=vec2(tc.x,tc.y+1*pwv);
    vec2 tc2=vec2(tc.x,tc.y+2*pwv);

    vec4 c=textureLod(tex,tc,lod)*6.0/21.0;
    c+=textureLod(tex,tc_1,lod)*4.0/21.0;
    c+=textureLod(tex,tc_2,lod)*1.0/21.0;
    c+=textureLod(tex,tc1,lod)*4.0/21.0;
    c+=textureLod(tex,tc2,lod)*6.0/21.0;
    return c;
}

void main(){


    fragColor = blurSelect();


    //fragColor.z=1;

}
