#!/bin/bash

# this creates strokes for gulshan algorithm

# Creates strokes from 3D Symmetry Map
# usage: ../bin/DrawStrokesForGulshan saliency.png output.txt result.png
#   saliency.png          ... saliency image
#   points.txt            ... output text file with points
#   result_base.png       ... output file name
#  Example: ../bin/DrawStrokesForGulshan saliency.png points.txt result.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- saliencyimages_path
#$5 -- points_path
#$6 -- outputimage_path
function calculateStrokesMapBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $4$i.png $5$i.txt $6$i.png"

      $3 $4$i.png $5$i.txt $6$i.png
    done 
} 

start_image=0
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/DrawStrokesForGulshan"
saliencyimages_path='/home/ekaterina/work/db/KLai/SYM3D/SYM3D_SIMPLE_SUM_LIN/'

points_path=$saliencyimages_path'TJ/cloud'
      
outputimage_path='/home/ekaterina/work/db/KLai/Gulshan/strokes'
mkdir $outputimage_path
outputimage_path=$outputimage_path/cloud
      
saliencyimages_path=$saliencyimages_path/'cloud'

calculateStrokesMapBatchProcess $start_image $end_image $exe_path $saliencyimages_path $points_path $outputimage_path



