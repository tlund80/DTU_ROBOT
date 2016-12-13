#!/bin/bash

base_path="/media/thso/Elements/scene_data/"
save_path="/media/thso/Elements/Object_pose_dataset/"
scene_name=""
FILECOUNT=0
DIRCOUNT=-1

for dir in /media/thso/Elements/scene_data/*
  do 
   # dir=${dir%*/}
    if [ -d "$dir" ]; then
        dirname=${dir##*/}
	echo "cd into ${dirname}"
        scene_num=(${dirname//scene_/ })
       # rm -r ${save_path}scenes/scene_${scene_num}
        #mkdir -p ${save_path}scenes/scene_${scene_num}/stl
	#mkdir -p ${save_path}scenes/scene_${scene_num}/stl/trfm
        #rm -r ${save_path}scenes/scene_${scene_num}/kinect
	#mkdir -p ${save_path}scenes/scene_${scene_num}/kinect
	#mkdir -p ${save_path}scenes/scene_${scene_num}/kinect/raw/
   
        #Copy stl clouds
        #cp ${dir}/pos_00/stl_0.ply ${save_path}scenes/scene_${scene_num}/stl/stl_00.ply
 	#cp ${dir}/pos_01/stl_1.ply ${save_path}scenes/scene_${scene_num}/stl/stl_01.ply
 	#cp ${dir}/pos_02/stl_2.ply ${save_path}scenes/scene_${scene_num}/stl/stl_02.ply
 	#cp ${dir}/pos_03/stl_3.ply ${save_path}scenes/scene_${scene_num}/stl/stl_03.ply
 	#cp ${dir}/pos_04/stl_4.ply ${save_path}scenes/scene_${scene_num}/stl/stl_04.ply
	#cp ${dir}/pos_05/stl_5.ply ${save_path}scenes/scene_${scene_num}/stl/stl_05.ply
 	#cp ${dir}/pos_06/stl_6.ply ${save_path}scenes/scene_${scene_num}/stl/stl_06.ply
 	#cp ${dir}/pos_07/stl_7.ply ${save_path}scenes/scene_${scene_num}/stl/stl_07.ply
 	#cp ${dir}/pos_08/stl_8.ply ${save_path}scenes/scene_${scene_num}/stl/stl_08.ply
 	#cp ${dir}/pos_09/stl_9.ply ${save_path}scenes/scene_${scene_num}/stl/stl_09.ply
 	#cp ${dir}/pos_10/stl_10.ply ${save_path}scenes/scene_${scene_num}/stl/stl_10.ply
        #cp ${dir}/full_stl.ply ${save_path}scenes/scene_${scene_num}/stl/stl_full.ply

	#Copy stl transformations
        #cp ${dir}/pos_00/stl_to_world_trfm_0.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_00.txt
        #cp ${dir}/pos_01/stl_to_world_trfm_1.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_01.txt
        #cp ${dir}/pos_02/stl_to_world_trfm_2.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_02.txt
        #cp ${dir}/pos_03/stl_to_world_trfm_3.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_03.txt
        #cp ${dir}/pos_04/stl_to_world_trfm_4.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_04.txt
        #cp ${dir}/pos_05/stl_to_world_trfm_5.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_05.txt
        #cp ${dir}/pos_06/stl_to_world_trfm_6.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_06.txt
        #cp ${dir}/pos_07/stl_to_world_trfm_7.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_07.txt
        #cp ${dir}/pos_08/stl_to_world_trfm_8.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_08.txt
        #cp ${dir}/pos_09/stl_to_world_trfm_9.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_09.txt
        #cp ${dir}/pos_10/stl_to_world_trfm_10.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_to_world_trfm_10.txt

	#Copy stl icp transformations
        cp ${dir}/pos_00/stl_icp_trfm_0.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_00.txt
        cp ${dir}/pos_01/stl_icp_trfm_1.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_01.txt
        cp ${dir}/pos_02/stl_icp_trfm_2.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_02.txt
        cp ${dir}/pos_03/stl_icp_trfm_3.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_03.txt
        cp ${dir}/pos_04/stl_icp_trfm_4.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_04.txt
        cp ${dir}/pos_05/stl_icp_trfm_5.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_05.txt
        cp ${dir}/pos_06/stl_icp_trfm_6.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_06.txt
        cp ${dir}/pos_07/stl_icp_trfm_7.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_07.txt
        cp ${dir}/pos_08/stl_icp_trfm_8.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_08.txt
        cp ${dir}/pos_09/stl_icp_trfm_9.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_09.txt
        cp ${dir}/pos_10/stl_icp_trfm_10.txt ${save_path}scenes/scene_${scene_num}/stl/trfm/stl_icp_trfm_10.txt

  	#Copy cropped kinect clouds
     #   cp ${dir}/pos_00/kinect_0.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_00.ply
 #	cp ${dir}/pos_01/kinect_1.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_01.ply
# 	cp ${dir}/pos_02/kinect_2.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_02.ply
# 	cp ${dir}/pos_03/kinect_3.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_03.ply
 #	cp ${dir}/pos_04/kinect_4.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_04.ply
#	cp ${dir}/pos_05/kinect_5.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_05.ply
 #	cp ${dir}/pos_06/kinect_6.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_06.ply
 #	cp ${dir}/pos_07/kinect_7.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_07.ply
 #	cp ${dir}/pos_08/kinect_8.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_08.ply
 #	cp ${dir}/pos_09/kinect_9.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_09.ply
 #	cp ${dir}/pos_10/kinect_10.ply ${save_path}scenes/scene_${scene_num}/kinect/kinect_10.ply
        #cp ${dir}/full_stl.ply ${save_path}scenes/scene_${scene_num}/stl/stl_full.ply


	#Copy raw kinect transformations
     #   cp ${dir}/pos_00/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_00.pcd
#	cp ${dir}/pos_00/kinect/kinect_0.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_00.ply
#	cp ${dir}/pos_00/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_00.png
#	cp ${dir}/pos_00/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_00.png
#        cp ${dir}/pos_01/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_01.pcd
#	cp ${dir}/pos_01/kinect/kinect_1.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_01.ply
#	cp ${dir}/pos_01/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_01.png
#	cp ${dir}/pos_01/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_01.png
#        cp ${dir}/pos_02/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_02.pcd
#	cp ${dir}/pos_02/kinect/kinect_2.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_02.ply
#	cp ${dir}/pos_02/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_02.png
#	cp ${dir}/pos_02/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_02.png
#   	cp ${dir}/pos_03/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_03.pcd
#	cp ${dir}/pos_03/kinect/kinect_3.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_03.ply
#	cp ${dir}/pos_03/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_03.png
#	cp ${dir}/pos_03/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_03.png
#   	cp ${dir}/pos_04/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_04.pcd
#	cp ${dir}/pos_04/kinect/kinect_4.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_04.ply
#	cp ${dir}/pos_04/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_04.png
#	cp ${dir}/pos_04/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_04.png
#   	cp ${dir}/pos_05/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_05.pcd
#	cp ${dir}/pos_05/kinect/kinect_5.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_05.ply
#	cp ${dir}/pos_05/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_05.png
#	cp ${dir}/pos_05/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_05.png
#   	cp ${dir}/pos_06/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_06.pcd
#	cp ${dir}/pos_06/kinect/kinect_6.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_06.ply
#	cp ${dir}/pos_06/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_06.png
#	cp ${dir}/pos_06/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_06.png
#   	cp ${dir}/pos_07/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_07.pcd
#	cp ${dir}/pos_07/kinect/kinect_7.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_07.ply
#	cp ${dir}/pos_07/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_07.png
#	cp ${dir}/pos_07/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_07.png
#   	cp ${dir}/pos_08/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_08.pcd
#	cp ${dir}/pos_08/kinect/kinect_8.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_08.ply
#	cp ${dir}/pos_08/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_08.png
#	cp ${dir}/pos_08/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_08.png
#   	cp ${dir}/pos_09/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_09.pcd
#	cp ${dir}/pos_09/kinect/kinect_9.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_09.ply
#	cp ${dir}/pos_09/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_09.png
#	cp ${dir}/pos_09/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_09.png
#   	cp ${dir}/pos_10/kinect/kinect_cloud.pcd ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_10.pcd
#	cp ${dir}/pos_10/kinect/kinect_10.ply ${save_path}scenes/scene_${scene_num}/kinect/raw/cloud_10.ply
#	cp ${dir}/pos_10/kinect/kinect_color.png ${save_path}scenes/scene_${scene_num}/kinect/raw/color_10.png
#	cp ${dir}/pos_10/kinect/kinect_depth_colored.png ${save_path}scenes/scene_${scene_num}/kinect/raw/depth_10.png



        cd ..
        
    fi
  done;




#echo $base_path
#./dtu_reconstruction -reconstruct /media/thso/My\ Passport/DTUData/scenes/scene_data/scene_054/ -stl

#for i in $(seq -f "%03g" 0 5)
#do
#   echo "Reconstruct scene $i"
#   scene_name="scene_$i" 
#   echo "execute: ../build/dtu_reconstruction -reconstruct $base_path$scene_name"/" -stl"
#   ../build/dtu_reconstruction -reconstruct $base_path$scene_name"/" -stl
#done
