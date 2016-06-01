#!/bin/bash
#base_path="/media/thso/Dataset1/DTUData/scenes/scene_data/"
base_path="/media/thso/Elements/scene_data/"
scene_name=""

for i in $(seq -f "%03g" $1 $2)
do
   scene_name="scene_$i" 
   echo "execute: ./dtu_reconstruction -reconstruct $base_path$scene_name"/" -kinect"
   ./dtu_reconstruction -reconstruct $base_path$scene_name"/" -kinect
done
