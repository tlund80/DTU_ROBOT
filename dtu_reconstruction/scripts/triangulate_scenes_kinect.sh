#!/bin/bash

base_path="/media/thso/Elements/scene_data"
dataset_path="/media/thso/Elements/Object_pose_dataset"
scene_name=""

for i in $(seq -f "%03g" $1 $2)
do
   scene_name="scene_$i" 
   mkdir -p /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/
   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_00.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_00.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_00.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_00.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_01.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_01.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_01.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_01.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_02.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_02.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_02.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_02.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_03.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_03.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_03.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_03.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_04.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_04.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_04.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_04.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_05.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_05.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_05.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_05.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_06.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_06.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_06.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_06.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_07.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_07.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_07.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_07.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_08.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_08.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_08.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_08.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_09.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_09.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_09.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_09.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_10.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_10.ply 1 1"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $dataset_path/scenes/$scene_name/kinect/kinect_10.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/kinect/mesh/kinect_10.ply 1 1

  
done
