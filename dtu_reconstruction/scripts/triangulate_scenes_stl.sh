#!/bin/bash

base_path="/media/thso/Elements/scene_data"
scene_name=""

for i in $(seq -f "%03g" $1 $2)
do
   scene_name="scene_$i" 
   mkdir -p /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/
   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_00/stl_0.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_0.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_00/stl_0.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_0.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_01/stl_1.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_1.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_01/stl_1.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_1.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_02/stl_2.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_2.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_02/stl_2.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_2.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_03/stl_3.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_3.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_03/stl_3.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_3.ply 1 1


   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_04/stl_4.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_4.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_04/stl_4.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_4.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_05/stl_5.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_5.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_05/stl_5.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_5.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_06/stl_6.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_6.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_06/stl_6.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_6.ply 1 1

   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_07/stl_7.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_7.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_07/stl_7.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_7.ply 1 1


   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_08/stl_8.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_8.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_08/stl_8.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_8.ply 1 1


   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_09/stl_9.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_9.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_09/stl_9.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_9.ply 1 1


   echo "execute: ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_10/stl_10.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_10.ply 1 1
done"
   ~/DTU_ROBOT/feature_matching_benchmark/build/delaunay $base_path/$scene_name/pos_10/stl_10.ply /media/thso/Elements/Object_pose_dataset/scenes/$scene_name/stl/mesh/stl_10.ply 1 1
done
