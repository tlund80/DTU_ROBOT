#!/bin/bash

########################################################################
# Test script for the UWA dataset, all models are in [mm] in this case #
########################################################################

# Test parameters
dec=0.125 # Decimation
rad=10,7.5,20,12.5,10,17.5,12.5 # Feature radius multiplier
res=5 # Feature resolution multiplier
thres=1 # Inlier threshold multipler, set to <= 0 for using 1 x mesh resolution

# Paths
exec_dir=build
exec_name=feature_matching_dataset
data_dir=~/workspace/datasets/uwa
output_dir=output/uwa

# Positionals
objects=`ls $data_dir/models/*.ply -v1 | grep -v rhino`
scenes=`ls $data_dir/scenes/*.ply -v1`
#scenes=`ls $data_dir/scenes/*.ply -v1 | grep 'rs1.ply'` # Fast test of first scene
pose_dir=$data_dir/GroundTruth_3Dscenes
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --threshold=$thres --metrics=L2_RATIO"
flags="--match-scene-objects --pose-file-model-first --verbose"

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $* 2> /dev/null

#gdb --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $*
