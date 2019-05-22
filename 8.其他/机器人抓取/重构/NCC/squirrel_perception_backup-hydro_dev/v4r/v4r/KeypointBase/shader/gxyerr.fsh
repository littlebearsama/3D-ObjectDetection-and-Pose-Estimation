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



in vec2 tc;
in vec2 tl;
in vec2 t;
in vec2 tr;
in vec2 l;
in vec2 r;
in vec2 bl;
in vec2 b;
in vec2 br;

uniform sampler2D tgtTexture;
uniform sampler2D srcTexture;
uniform int sqrt_maxPatchCount;//needed to destinguish the black pixel

out vec3 g;
out vec2 err;


/*
 * Generate first the soebel image for all the patches and calculates afterwards the correction terms.
 */
void main(){

    ivec2 texRes=textureSize(srcTexture,0);
    float pw=1.0/float(texRes.x);//pixel width


    float I1   = texture(tgtTexture,tc).x*1;
    float I1tl = texture(tgtTexture,tl).x*1;
    float I1t  = texture(tgtTexture,t).x*1;
    float I1tr = texture(tgtTexture,tr).x*1;
    float I1l  = texture(tgtTexture,l).x*1;
    float I1r  = texture(tgtTexture,r).x*1;
    float I1bl = texture(tgtTexture,bl).x*1;
    float I1b  = texture(tgtTexture,b).x*1;
    float I1br = texture(tgtTexture,br).x*1;

    //calculate derivatives
    float dx1=-I1tl  -2*I1l - I1bl + I1tr + 2*I1r + I1br;
    float dy1=-I1tl  -2*I1t - I1tr + I1br + 2*I1b + I1br;


    float I2   = texture(srcTexture,tc).x*1;
    float I2tl = texture(srcTexture,tl).x*1;
    float I2t  = texture(srcTexture,t).x*1;
    float I2tr = texture(srcTexture,tr).x*1;
    float I2l  = texture(srcTexture,l).x*1;
    float I2r  = texture(srcTexture,r).x*1;
    float I2bl = texture(srcTexture,bl).x*1;
    float I2b  = texture(srcTexture,b).x*1;
    float I2br = texture(srcTexture,br).x*1;

    //calculate derivatives
    float dx2=-I2tl  -2*I2l - I2bl + I2tr + 2*I2r + I2br;
    float dy2=-I2tl  -2*I2t - I2tr + I2br + 2*I2b + I2br;


    float dx=(dx1+dx2)*255;
    float dy=(dy1+dy2)*255;

    //calculate gxx, gxy and gyy (sum is done by mipmapping)
    float gxx=dx*dx;
    float gxy=dx*dy;
    float gyy=dy*dy;
    g = vec3(gxx,gxy,gyy);


    //calc the error (sum is done by mipmapping)
    float diff=(I1-I2)*255;
    float errx=diff*dx;
    float erry=diff*dy;
    err = vec2(errx,erry);


    // generate borders of 0 around each patch...... (to define a region of interest)
    float patchSize=texRes.x/sqrt_maxPatchCount;
    vec2 texPos=tc*texRes.x;
    float modx = mod(texPos.x+1,patchSize);
    if (modx<2){
        g= vec3(0);
        err=vec2(0);
    }
    float mody = mod(texPos.y+1,patchSize);
    if (mody<2){
        g= vec3(0);
        err=vec2(0);
    }

}
