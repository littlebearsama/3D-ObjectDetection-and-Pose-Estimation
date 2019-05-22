#!/bin/bash

# this script calculated Height Saliency Maps

#usage: ../bin/ExmplSurfOrinetationMap image.png cloud.pcd type pyramid_type combination_type normalization_type
#  image.png             ... color image
#  cloud.pcd             ... point cloud
#  type                  ... 0 -- horizontal; 1 -- vertical
#  pyramid_type          ... 0 -- no pyramid; 1 -- simple pyramid; 2 -- Itti pyramid; 3 -- Frintrop pyramid
#  combination_type      ... 0 -- SUM; 1 -- MUL; 2 -- MAX
#  normalization_type    ... 0 -- LIN; 1 -- NMS; 2 -- NLM
# Example: ../bin/ExmplSurfOrinetationMap image.png cloud.pcd 0 0 0 0

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- pointclouds_path
#$5 -- rgbimages_path
#$6 -- results_path
#$7 -- add_to_name
#$8 -- type
#$9 -- pyramid_type
#$10 -- combination_type
#$11 -- normalization_type
function calculateSurfaceOrientationMapBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $5$i.png $4$i$7.pcd $8 $9 ${10} ${11} $6$i.png"

      $3 $5$i.png $4$i$7.pcd $8 $9 ${10} ${11} $6$i.png
    done 
} 

start_image=0
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/ExmplSurfOrientationMap"
pointclouds_path='/home/ekaterina/work/db/KLai/ConfDiffused_ascii/cloud'
rgbimages_path='/home/ekaterina/work/db/KLai/rgb/cloud'
add_to_name="ConfDiffused.Cloud"
results_path_base='/home/ekaterina/work/db/KLai/RSO/RSO'

type=0

for pyramid_type in 0 1 2 #0 1 2
do
  if [ "$pyramid_type" -eq "0" ]
  then
    pyramid_type_txt='SINGLE'
  fi
  if [ "$pyramid_type" -eq "1" ]
  then
    pyramid_type_txt='SIMPLE'
  fi
  if [ "$pyramid_type" -eq "2" ]
  then
    pyramid_type_txt='ITTI'
  fi
  
  for combination_type in 0 2 #2
  do
    if [ "$combination_type" -eq "0" ]
    then
      combination_type_txt='SUM'
    fi
    if [ "$combination_type" -eq "2" ]
    then
      combination_type_txt='MAX'
    fi
    
    for normalization_type in 0 1 2 #0 1 2
    do
      if [ "$normalization_type" -eq "0" ]
      then
        normalization_type_txt='LIN'
      fi
      if [ "$normalization_type" -eq "1" ]
      then
        normalization_type_txt='NMS'
      fi
      if [ "$normalization_type" -eq "2" ]
      then
        normalization_type_txt='NLM'
      fi
      
      results_path=$results_path_base'_'$pyramid_type_txt'_'$combination_type_txt'_'$normalization_type_txt
      mkdir $results_path
      results_path=$results_path/cloud

      calculateSurfaceOrientationMapBatchProcess $start_image $end_image $exe_path $pointclouds_path $rgbimages_path $results_path $add_to_name $type $pyramid_type $combination_type $normalization_type
      
    done
  done
done