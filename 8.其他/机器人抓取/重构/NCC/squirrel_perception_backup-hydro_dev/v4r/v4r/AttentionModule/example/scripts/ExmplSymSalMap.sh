#!/bin/bash

# this script calculated Height Saliency Maps

#usage: ../bin/ExmplSymSalMap image.png result.png
#  image.png             ... color image
#  result.png            ... output file name
# Example: ../bin/ExmplSymSalMap image.png result.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- rgbimages_path
#$5 -- results_path
function calculateSymSaliencyMapBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo $5$i.png

      $3 $4$i.png $5$i.png
    done 
} 

start_image=0
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/ExmplSymSalMap"
rgbimages_path='/home/ekaterina/work/db/KLai/rgb/cloud'

results_path='/home/ekaterina/work/db/KLai/SAL2D/SYM2D'
mkdir $results_path
results_path=$results_path/cloud

calculateSymSaliencyMapBatchProcess $start_image $end_image $exe_path $rgbimages_path $results_path