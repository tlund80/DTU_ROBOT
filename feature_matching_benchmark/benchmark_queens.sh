#!/bin/bash

#######################################
# Test script for the Queen's dataset #
#######################################

# Test parameters
dec=0.75 # Decimation
rad=10,7.5,22.5,12.5,12.5,20,12.5,7.5,20 # Feature radius multiplier
res=5 # Feature resolution multiplier
thres=1 # Inlier threshold multipler, set to <= 0 for using 1 x mesh resolution

# Paths
exec_dir=build
exec_name=feature_matching_dataset
data_dir=~/workspace/datasets/queens
output_dir=output/queens

# Positionals
objects=`ls $data_dir/models_reconstructed/*.ply -v1`
scenes=`ls $data_dir/lidar_point_clouds_reconstructed/*.ply -v1`
pose_dir=$data_dir/ground_truth_poses
pose_separator="_"
pose_suffix=txt

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --threshold=$thres --metrics L2_RATIO"
flags="--match-scene-objects --verbose " #--correspondences

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $* 2> /dev/null

#gdb --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $*
