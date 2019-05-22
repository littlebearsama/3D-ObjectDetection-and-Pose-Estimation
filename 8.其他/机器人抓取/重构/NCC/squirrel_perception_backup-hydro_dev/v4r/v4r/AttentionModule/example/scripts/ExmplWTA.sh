#!/bin/bash

# this script calculated WTA

#usage: %s image.png saliency.png output.txt output.png useRandom useCentering useMorphologyOpenning\n"
#  image.png             ... color image
#  saliency.png          ... saliency image
#  points.txt            ... output text file with points
#  points.png            ... output image with points
#  useRandom             ... 0 -- false; 1 -- true;
#  useCentering          ... 0 -- false; 1 -- true;
#  useMorphologyOpenning ... 0 -- false; 1 -- true;
# Example: %s 1 image.png saliency.png points.txt points.png 0 0 0

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- rgbimages_path
#$5 -- saliencyimages_path
#$6 -- points_path
#$7 -- outputimage_path
#$8 -- useRandom
#$9 -- useCentering
#$10 -- useMorphologyOpenning
function calculateWTABatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $4$i.png $5$i.png $6$i.txt $7$i.png $8 $9 ${10}"

      $3 $4$i.png $5$i.png $6$i.txt $7$i.png $8 $9 ${10}
    done 
} 

start_image=66
end_image=86
exe_path="/home/ekaterina/work/trunk/bin/ExmplWTA"
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
  
  for combination_type in 0 #0 2 # 0 2
  do
    if [ "$combination_type" -eq "0" ]
    then
      combination_type_txt='SUM'
    fi
    if [ "$combination_type" -eq "2" ]
    then
      combination_type_txt='MAX'
    fi
    
    for normalization_type in 0 #0 1 2 #0 1 2
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

      points_path=$saliencyimages_path/'WTA'
      mkdir $points_path
      points_path=$points_path/cloud

      outputimage_path=$saliencyimages_path/'WTA_images'
      mkdir $outputimage_path
      outputimage_path=$outputimage_path/cloud
      
      saliencyimages_path=$saliencyimages_path/'cloud'

      calculateWTABatchProcess $start_image $end_image $exe_path $rgbimages_path $saliencyimages_path $points_path $outputimage_path 0 1 0

    done
  done
done

# saliencyimages_path_base='/home/ekaterina/work/db/KLai/SAL2D'
# 
# for i in SYM2D #WANG WANG_HOU WANG_ITTI WANG_BRUCE
# do
# 
#   points_path=$saliencyimages_path_base/$i/'WTA'
#   mkdir $points_path
#   points_path=$points_path/'cloud'
#   
#   outputimage_path=$saliencyimages_path_base/$i/'WTA_images'
#   mkdir $outputimage_path
#   outputimage_path=$outputimage_path/'cloud'
#   
#   
#   calculateWTABatchProcess $start_image $end_image $exe_path $rgbimages_path $saliencyimages_path_base/$i/'cloud' $points_path $outputimage_path 0 1 0
# done


