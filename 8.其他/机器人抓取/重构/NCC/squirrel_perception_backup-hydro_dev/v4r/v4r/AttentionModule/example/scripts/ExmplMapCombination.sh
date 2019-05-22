#!/bin/bash

# Calculates 3D Symmentry Saliency Map

# usage: ../bin/ExmplMapCombination image.png cloud.pcd combination_type normalization_type result.png
#   image.png             ... color image
#   cloud.pcd             ... point cloud
#   combination_type      ... 0 -- SUM; 1 -- MUL; 2 -- MAX
#   normalization_type    ... 0 -- LIN; 1 -- NMS; 2 -- NLM
#   result.png            ... output file name
#  Example: ../bin/ExmplMapCombination image.png cloud.pcd 0 0 result.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- pointclouds_path
#$5 -- rgbimages_path
#$6 -- results_path
#$7 -- add_to_name
#$8 -- combination_type
#$9 -- normalization_type
#10 -- color saliency map
function calculateMapCombinationProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $5$i.png $4$i$7.pcd $8 $9 $6$i.png"

      $3 $5$i.png $4$i$7.pcd $8 $9 $6$i.png ${10}$i.png
    done 
} 

start_image=0
end_image=91
exe_path="/home/ekaterina/work/trunk/bin/ExmplMapCombination"
pointclouds_path='/home/ekaterina/work/db/TOSD-2.0/ConfDiffused_ascii/test'
rgbimages_path='/home/ekaterina/work/db/TOSD-2.0/rgb/test'
add_to_name="ConfDiffused.Cloud"

color_saliency_maps='/home/ekaterina/work/db/TOSD-2.0/SAL2D/HAREL/test'

results_path='/home/ekaterina/work/db/TOSD-2.0/COMB/COMB_COLOR_'

for combination_type in 0 2
do
  for normalization_type in 0 1 2
  do 
    if [ "$combination_type" -eq "0" ]
    then
      combination_type_txt='SUM'
    fi
  
    if (( $combination_type==2 )); then
      combination_type_txt='MAX'
    fi
  
    if (( $normalization_type==0 )); then
      normalization_type_txt='LIN'
    fi
  
    if (( $normalization_type==1 )); then
      normalization_type_txt='NMS'
    fi
  
    if (( $normalization_type==2 )); then
      normalization_type_txt='NLM'
    fi
  
  outputimage_path=$results_path$combination_type_txt'_'$normalization_type_txt

  mkdir $outputimage_path
  outputimage_path=$outputimage_path/test

  calculateMapCombinationProcess $start_image $end_image $exe_path $pointclouds_path $rgbimages_path $outputimage_path $add_to_name $combination_type $normalization_type $color_saliency_maps
  
  done 
done




