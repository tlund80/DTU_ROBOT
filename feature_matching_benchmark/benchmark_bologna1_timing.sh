#!/bin/bash

# Test parameters
rad=0.01 # Feature radius multiplier
thres=-1 # Inlier threshold multipler, set to <= 0 for using 1 x mesh resolution

# Paths
exec_dir=build
exec_name=feature_matching_timing
data_dir=~/workspace/datasets/bologna
output_dir=output/bologna1_timing

# Positionals
scenes=`ls $data_dir/dataset1_scenes_timing/*.ply -v1`
pose_dir=$data_dir/dataset1_scenes_timing
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--radius=$rad "
flags="--verbose"

# Start
$exec_dir/$exec_name "$scenes" $output_dir $options $flags $* #2> /dev/null

#gdb --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $flags $*
