#!/bin/bash

##########################################################################
# Test script for the Bologna 1 dataset which contains noise-free scenes #
##########################################################################

# Test parameters
dec=-1 # Decimation
rad=30,20,30,30,30,30,30 # Feature radius multiplier
res=6 # Feature resolution multiplier
thres=1 # Inlier threshold multipler, set to <= 0 for using 1 x mesh resolution

noise_scn=0.3 # Scene-only noise level multipler [0.1 0.3]

# Paths
exec_dir=build
exec_name=feature_matching_dataset
data_dir=~/workspace/datasets/bologna
output_dir=output/bologna1_noise"$noise_scn"

# Positionals
objects=`ls $data_dir/dataset1-2_models/3D_models/Stanford/*.ply -v1`
scenes=`ls $data_dir/dataset1_scenes_n$noise_scn/*.ply -v1`
pose_dir=$data_dir/dataset1_scenes/3D_models/Stanford/Random
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --threshold=$thres --metrics=L2_RATIO"
flags="--verbose"

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $* #2> /dev/null

#gdb --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $*
