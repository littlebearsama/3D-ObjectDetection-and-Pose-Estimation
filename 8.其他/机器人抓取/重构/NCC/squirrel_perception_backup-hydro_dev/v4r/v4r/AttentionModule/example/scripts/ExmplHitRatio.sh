#!/bin/bash

# this script calculates Hit Ratio

# Calculates HitRatio from attention points
# usage: ../bin/ExmplHitRatio points.txt mask.png hitratio.txt
#   points.txt            ... input text file with points
#   mask.png              ... mask image
#   hitratio.txt          ... output text file with hitratios
#  Example: ../bin/ExmplHitRatio points.txt mask.png hitratio.txt

#$1 -- start_image
#$2 -- end_image
#$3 -- exe_path
#$4 -- attention_points_path
#$5 -- maskimages_path
#$6 -- hitratio_path
function calculateHitRatioBatchProcess {
    for (( i=$1; i<=$2; i=i+1 ))
    do
      echo "$3 $4$i.txt $5$i.png $6$i.txt"

      $3 $4$i.txt $5$i.png $6$i.txt
    done 
} 

start_image=0
end_image=91
exe_path="/home/ekaterina/work/trunk/bin/ExmplHitRatio"
maskimages_path='/home/ekaterina/work/db/TOSD-2.0/gt/test'
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
#       hitratio_path=$points_path_base'_'$pyramid_type_txt'_'$combination_type_txt'_'$normalization_type_txt/$points_type_txt'_hitratio'
#       mkdir $hitratio_path
#       hitratio_path=$hitratio_path/test
# 
#       calculateHitRatioBatchProcess $start_image $end_image $exe_path $attention_points_path $maskimages_path $hitratio_path
# 
#     done
#   done
# done

for i in WANG WANG_HOU WANG_ITTI WANG_BRUCE
do

  points_path_base='/home/ekaterina/work/db/TOSD-2.0/SAL2D/'$i
  hitratio_path=$points_path_base/'MSR_hitratio'
  mkdir $hitratio_path
  hitratio_path=$hitratio_path/'test'
  attention_points_path=$points_path_base/'MSR/test'
  calculateHitRatioBatchProcess $start_image $end_image $exe_path $attention_points_path $maskimages_path $hitratio_path

  hitratio_path=$points_path_base/'WTA_hitratio'
  mkdir $hitratio_path
  hitratio_path=$hitratio_path/'test'
  attention_points_path=$points_path_base/'WTA/test'
  calculateHitRatioBatchProcess $start_image $end_image $exe_path $attention_points_path $maskimages_path $hitratio_path
  
done

