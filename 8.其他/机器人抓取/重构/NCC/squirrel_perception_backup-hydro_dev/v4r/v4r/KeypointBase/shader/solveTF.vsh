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

//layout (location = 1) in int index;
//layout (location = 0) in vec3 pos;
in int index;
in vec3 pos;

uniform float lod; //mipmap level needet to read the average of gxx..... and err
uniform int pyrLvl;//pyramid level we are in.
uniform float minDeterminant;
uniform float minDisplacement;
uniform float stepFactor;
uniform float sqrt_maxPatchCount;
uniform vec2 tgtImRes;

uniform sampler2D gxyTexture;

uniform sampler2D errxyTexture;

//output of transform feedback.
out vec3 transformOut;

/*
 * Transform feedback vertex shader correcting the keypoint position:
 */
void main(){

    transformOut=pos;
    float converged=pos.z;
    if (converged <0){ //if it has proven to not convergate TODO write this different
        return;
    }

    //test if patch is hitting the border
    ivec2 texRes=textureSize(gxyTexture,0);
    float patchSize=texRes.x/sqrt_maxPatchCount;

    float scale = pow(2,pyrLvl);
    if (  pos.x < 0.0f || tgtImRes.x-(pos.x+patchSize*scale) < 1.001 ||
          pos.y < 0.0f || tgtImRes.y-(pos.y+patchSize*scale) < 1.001 ) {
      if (pyrLvl==0){ //if we are on the base level and still hitting the border we return an error
        converged = -1;
      }
      transformOut.z=converged;
      return;
    }

    //get the pixel, to the accoarding index we are working on:
    float col = float(mod(float(index),sqrt_maxPatchCount))/float(sqrt_maxPatchCount);
    float row = float(floor(float(index)/sqrt_maxPatchCount))/float(sqrt_maxPatchCount);

    vec2 tc=vec2(col,row)+vec2(0.5/float(sqrt_maxPatchCount));
    vec2 err = -textureLod(errxyTexture,tc,lod).xy*patchSize*patchSize*stepFactor;
    vec3 gxxxyyy = textureLod(gxyTexture,tc,lod).xyz*patchSize*patchSize;

    float gxx=gxxxyyy.x;
    float gxy=gxxxyyy.y;
    float gyy=gxxxyyy.z;

    float det = gxx*gyy - gxy*gxy;

    //If the determinant is to small, we simply do not attempt to correct
    if (det < minDeterminant)  {
        converged =-2;
        transformOut.z=converged;
        return;
    }

    vec2 delta;
    delta.x = (gyy*err.x - gxy*err.y)/det;
    delta.y = (gxx*err.y - gxy*err.x)/det;

    transformOut.xy += delta;
    transformOut.z=converged;

    return;

}
