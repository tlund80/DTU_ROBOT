#!/bin/bash

#######################################
# Test script for the Queen's dataset #
#######################################

# General
dec=-1 # Decimation

# Features
features=ecsad,ndhist,rops,shot,si
rad=25,25,20,22.5,25 # Feature radius multiplier
res=5 # Feature resolution multiplier, scenes
resq=2.5 # Feature resolution multiplier, objects

# Recognition
ransac_iter=1000 # RANSAC iterations
corr_frac=0.5 # Fraction of best correspondences to include in RANSAC
inlier_frac=0.05 # Required inlier fraction for RANSAC
icp_iter=50 # ICP iterations
thres=5 # Inlier threshold multiplier, RANSAC+ICP, set to <= 0 for using 1 x mesh resolution

# Pose tolerances (metric and degrees)
trans_tol=50
rot_tol=7.5

# Paths
exec_dir=build
exec_name=feature_matching_recognition
data_dir=~/workspace/datasets/rgbd
output_dir=output/rgbd_rec

# Positionals
queries=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 1'`) # Odd indices
targets=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 0'`) # Even
	#queries=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 1' | grep table_1_31`) # Odd indices
	#targets=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 0' | grep table_1_36`) # Even
pose_dir=$data_dir/ground-truth
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --resolution-query=$resq --threshold=$thres --features=$features"
options_rec="--ransac-iterations=$ransac_iter --inlier-fraction=$inlier_frac --icp-iterations=$icp_iter --correspondence-fraction=$corr_frac --translation-tolerance=$trans_tol --rotation-tolerance=$rot_tol"
flags="--pose-file-model-first --append --pca --verbose"


# Start - use newline as IFS
ifsbak=$IFS
IFS=$'\n'
idxend=`expr ${#queries[@]} - 1`
for idx in `seq 0 $idxend`
do
	echo -------------------- TESTING SCENE INDEX $idx/$idxend --------------------
	query=${queries[$idx]}
	target=${targets[$idx]}
	if [ "$query" = "" ]; then break; fi # Handle final newline

    # Restore IFS
    IFS=$ifsbak
	
	$exec_dir/$exec_name $query $target $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 2> /dev/null
	
	#gdb -ex run --args $exec_dir/$exec_name $query $target $pose_dir $pose_suffix $output_dir $options $options_rec $flags $*
	
	stat=$?
	
    IFS=$'\n'
    
    if [ $stat -ne 0 ]; then echo "Wrong return code! Aborting..."; exit 1; fi
done

# Restore IFS
IFS=$ifsbak
