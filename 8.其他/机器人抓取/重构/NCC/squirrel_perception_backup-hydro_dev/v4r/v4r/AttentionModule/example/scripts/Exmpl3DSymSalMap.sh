#!/bin/bash

# this script calculated 3D Symmetry Saliency Maps

#usage: ../bin/Exmpl3DSymSalMap image.png cloud.pcd pyramid_type combination_type normalization_type result.png
#  image.png             ... color image
#  cloud.pcd             ... point cloud
#  pyramid_type          ... 0 -- no pyramid; 1 -- simple pyramid; 2 -- Itti pyramid; 3 -- Frintrop pyramid
#  combination_type      ... 0 -- SUM; 1 -- MUL; 2 -- MAX
#  normalization_type    ... 0 -- LIN; 1 -- NMS; 2 -- NLM
#  result.png            ... output file name
# Example: ../bin/ExmplSurfOrinetationMap image.png cloud.pcd 0 0 0 result.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- pointclouds_path
#$5 -- rgbimages_path
#$6 -- results_path
#$7 -- add_to_name
#$8 -- pyramid_type
#$9 -- combination_type
#$10 -- normalization_type
function calculate3DSymmetryMapBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $5$i.png $4$i$7.pcd $8 $9 ${10} $6$i.png"

      $3 $5$i.png $4$i$7.pcd $8 $9 ${10} $6$i.png
    done 
} 

start_image=66
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/Exmpl3DSymSalMap"
pointclouds_path='/home/ekaterina/work/db/KLai/ConfDiffused_ascii/cloud'
rgbimages_path='/home/ekaterina/work/db/KLai/rgb/cloud'
add_to_name="ConfDiffused.Cloud"
results_path_base='/home/ekaterina/work/db/KLai/SYM3D/SYM3D'

for pyramid_type in 1 #0 1 2
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
  
  for combination_type in 0 #0 2
  do
    if [ "$combination_type" -eq "0" ]
    then
      combination_type_txt='SUM'
    fi
    if [ "$combination_type" -eq "2" ]
    then
      combination_type_txt='MAX'
    fi
    
    for normalization_type in 0 #0 1 2
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

      calculate3DSymmetryMapBatchProcess $start_image $end_image $exe_path $pointclouds_path $rgbimages_path $results_path $add_to_name  $pyramid_type $combination_type $normalization_type

    done
  done
done