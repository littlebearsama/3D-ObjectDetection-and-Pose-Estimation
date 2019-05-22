#!/bin/bash

# this script draws attention points

#  Example: ../bin/ExmplDrawPoints Usage: points.txt image.png points.png

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- attention_points_path
#$5 -- rgbimages_path
#$6 -- resultimages_path
function drawPoints {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $4$i.txt $5$i.png $6$i.png"

      $3 $4$i.txt $5$i.png $6$i.png
    done 
} 

start_image=0
end_image=91
exe_path="/home/ekaterina/work/trunk/bin/ExmplDrawPoints"
rgbimages_path='/home/ekaterina/work/db/TOSD-2.0/rgb/test'
# points_path_base='/home/ekaterina/work/db/TOSD-2.0/SYM3D/SYM3D'
# 
# for pyramid_type in 0 1 2 #0 1 2
# do
#   if [ "$pyramid_type" -eq "0" ]
#   then
#     pyramid_type_txt='SINGLE'
#   fi
#   if [ "$pyramid_type" -eq "1" ]
#   then
#     pyramid_type_txt='SIMPLE'
#   fi
#   if [ "$pyramid_type" -eq "2" ]
#   then
#     pyramid_type_txt='ITTI'
#   fi
#   
#   for combination_type in 0 2 # 0 2
#   do
#     if [ "$combination_type" -eq "0" ]
#     then
#       combination_type_txt='SUM'
#     fi
#     if [ "$combination_type" -eq "2" ]
#     then
#       combination_type_txt='MAX'
#     fi
#     
#     for normalization_type in 0 1 2 #0 1 2
#     do
#       if [ "$normalization_type" -eq "0" ]
#       then
#         normalization_type_txt='LIN'
#       fi
#       if [ "$normalization_type" -eq "1" ]
#       then
#         normalization_type_txt='NMS'
#       fi
#       if [ "$normalization_type" -eq "2" ]
#       then
#         normalization_type_txt='NLM'
#       fi
#       
#       for points_type in 2 #0 1 2
#       do
#         if [ "$points_type" -eq "0" ]
#         then
#           points_type_txt='MSR'
#         fi
#         if [ "$points_type" -eq "1" ]
#         then
#           points_type_txt='WTA'
#         fi
#         if [ "$points_type" -eq "2" ]
#         then
#           points_type_txt='TJ'
#         fi
#       done
#       
#       attention_points_path=$points_path_base'_'$pyramid_type_txt'_'$combination_type_txt'_'$normalization_type_txt/$points_type_txt/'test'
# 
#       resultimages_path=$points_path_base'_'$pyramid_type_txt'_'$combination_type_txt'_'$normalization_type_txt/$points_type_txt'_images/test'
#       drawPoints $start_image $end_image $exe_path $attention_points_path $rgbimages_path $resultimages_path
# 
#     done
#   done
# done

#for i in AIM  HAREL  HAREL_fast  HOU  ITTI  SYM2D  WANG  WANG_BRUCE  WANG_HOU  WANG_ITTI
for i in COMB_COLOR_MAX_LIN COMB_COLOR_SUM_NLM COMB_SINGLE_MAX_NMS COMB_COLOR_MAX_NLM COMB_COLOR_SUM_NMS COMB_SINGLE_SUM_LIN COMB_COLOR_MAX_NMS COMB_SINGLE_MAX_LIN COMB_SINGLE_SUM_NLM COMB_COLOR_SUM_LIN COMB_SINGLE_MAX_NLM COMB_SINGLE_SUM_NMS 
do

  points_path_base='/home/ekaterina/work/db/TOSD-2.0/COMB/'$i
  
  attention_points_path=$points_path_base/'MSR/test'
  resultimages_path=$points_path_base/'MSR_images/test'
  drawPoints $start_image $end_image $exe_path $attention_points_path $rgbimages_path $resultimages_path

  attention_points_path=$points_path_base/'WTA/test'
  resultimages_path=$points_path_base/'WTA_images/test'
  drawPoints $start_image $end_image $exe_path $attention_points_path $rgbimages_path $resultimages_path
  
done

