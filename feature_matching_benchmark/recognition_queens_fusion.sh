#!/bin/bash

#######################################
# Test script for the Queen's dataset #
#######################################

# General
dec=0.75 # Decimation

# Features
features=ecsad,ndhist,rops,shot,si
rad=10,22.5,12.5,12.5,20
res=5 # Feature resolution multiplier, scenes
resq=2.5 # Feature resolution multiplier, objects

# Recognition
ransac_iter=1000 # RANSAC iterations
corr_frac=0.5 # Fraction of best correspondences to include in RANSAC
inlier_frac=0.05 # Required inlier fraction for RANSAC
icp_iter=50 # ICP iterations
thres=5 # Inlier threshold multiplier, RANSAC+ICP, set to <= 0 for using 1 x mesh resolution

# Pose tolerances (metric and degrees)
trans_tol=0.05
rot_tol=7.5

# Paths
exec_dir=build
exec_name=feature_matching_recognition
data_dir=~/workspace/datasets/queens
output_dir=output/queens_rec_fusion

# Positionals
objects=`ls $data_dir/models_reconstructed/*.ply -v1`
scenes=`ls $data_dir/lidar_point_clouds_reconstructed/*.ply -v1`
pose_dir=$data_dir/ground_truth_poses
pose_separator="_"
pose_suffix=txt

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --resolution-query=$resq --threshold=$thres --features=$features"
options_rec="--ransac-iterations=$ransac_iter --inlier-fraction=$inlier_frac --icp-iterations=$icp_iter --correspondence-fraction=$corr_frac --translation-tolerance=$trans_tol --rotation-tolerance=$rot_tol"
flags="--fusion --pca --verbose"

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 2> /dev/null

#gdb -ex r --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 2> /dev/null
