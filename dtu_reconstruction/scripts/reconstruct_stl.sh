#!/bin/bash

base_path="/media/thso/My\ Passport/DTUData/scenes/scene_data/"
scene_name=""

#echo $base_path
#./dtu_reconstruction -reconstruct /media/thso/My\ Passport/DTUData/scenes/scene_data/scene_054/ -stl

for i in $(seq -f "%03g" 0 5)
do
   echo "Reconstruct scene $i"
   scene_name="scene_$i" 
   echo "execute: ../build/dtu_reconstruction -reconstruct $base_path$scene_name"/" -stl"
   ../build/dtu_reconstruction -reconstruct $base_path$scene_name"/" -stl
done
