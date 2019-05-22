#!/bin/bash
point_clouds_directory='/home/ekaterina/work/db/KLai/ConfDiffused_ascii'
disparity_directory='/home/ekaterina/work/db/KLai/ConfDiffusedDisparity'

for (( i=0; i<=86; i++ ))
do

  echo $i
  
  ../../../../bin/ExmplXYZRGB2Disparity $point_clouds_directory/'cloud'$i'ConfDiffused.Cloud.pcd' $disparity_directory/'cloud'$i'.png' 640 480

done