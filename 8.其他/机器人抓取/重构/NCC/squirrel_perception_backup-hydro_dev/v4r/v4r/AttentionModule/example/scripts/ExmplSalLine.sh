#!/bin/bash

# this script calculated TJ

# Extracts attention points using TJ
#   usage: ../bin/ExmplSalLine image.png saliency.png output.txt result.png
#     image.png             ... color image
#     saliency.png          ... saliency image
#     points.txt            ... output text file with points
#     result.png            ... output file name
# Example: ../bin/ExmplSalLine 1 image.png saliency.png points.txt result.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- rgbimages_path
#$5 -- saliencyimages_path
#$6 -- points_path
#$7 -- outputimage_path
function calculateSalMapBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $4$i.png $5$i.png $6$i.txt $7$i.png"

      $3 $4$i.png $5$i.png $6$i.txt $7$i.png
    done 
} 

start_image=66
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/ExmplSalLine"
rgbimages_path='/home/ekaterina/work/db/KLai/rgb/cloud'
saliencyimages_path_base='/home/ekaterina/work/db/KLai/SYM3D/SYM3D'

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
  
  for combination_type in 0 # 0 2
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
      
      saliencyimages_path=$saliencyimages_path_base'_'$pyramid_type_txt'_'$combination_type_txt'_'$normalization_type_txt

      points_path=$saliencyimages_path/'TJ'
      mkdir $points_path
      points_path=$points_path/cloud

      outputimage_path=$saliencyimages_path/'TJ_images'
      mkdir $outputimage_path
      outputimage_path=$outputimage_path/cloud
      
      saliencyimages_path=$saliencyimages_path/'cloud'

      calculateSalMapBatchProcess $start_image $end_image $exe_path $rgbimages_path $saliencyimages_path $points_path $outputimage_path

    done
  done
done




