#!/bin/bash

base_path="/media/thso/Dataset1/models"
scene_name=""

#for i in $(seq -f "%03g" $1 $2)
dirlist=$(find $base_path -mindepth 1 -maxdepth 1 -type d)
for dir in $dirlist
do
 #  echo ${dir##*/}
   scene_name=${dir##*/} 
   echo "execute: dti_triangulation $base_path/$scene_name/full/"$scene_name"_cloud.ply  $base_path/$scene_name/full/"$scene_name"_mesh.ply --algorithm Poisson --poisson_depth 12"
   dti_triangulation $base_path/$scene_name/full/"$scene_name"_cloud.ply  $base_path/$scene_name/full/"$scene_name"_mesh.ply --algorithm Poisson --poisson_depth 12
  
done
