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



in vec2 tc;
in vec2 tl;
in vec2 t;
in vec2 tr;
in vec2 l;
in vec2 r;
in vec2 bl;
in vec2 b;
in vec2 br;
uniform sampler2D intensityTexture;

out vec4 fragColor;


void main(){

    float I1   = texture(intensityTexture,tc).x*1;
    float I1tl = texture(intensityTexture,tl).x*1;
    float I1t  = texture(intensityTexture,t).x*1;
    float I1tr = texture(intensityTexture,tr).x*1;
    float I1l  = texture(intensityTexture,l).x*1;
    float I1r  = texture(intensityTexture,r).x*1;
    float I1bl = texture(intensityTexture,bl).x*1;
    float I1b  = texture(intensityTexture,b).x*1;
    float I1br = texture(intensityTexture,br).x*1;

    //calculate derivatives
    float dx1=-I1tl  -2*I1l - I1bl + I1tr + 2*I1r + I1br;
    float dy1=-I1tl  -2*I1t - I1tr + I1br + 2*I1b + I1br;

    fragColor = vec4(I1,dx1,dy1,0);
    //ragColor = vec4(0,0,1,0);
    //fragColor = vec4(float(varIndex)/100,0,0,1);
}
