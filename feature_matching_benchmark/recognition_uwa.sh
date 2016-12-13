#!/bin/bash

########################################################################
# Test script for the UWA dataset, all models are in [mm] in this case #
########################################################################

# General
dec=0.125 # Decimation

# Features
features=ecsad,ndhist,rops #,shot,si
rad=10,20,12.5 #,17.5,10
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
data_dir=~/workspace/datasets/uwa
output_dir=output/uwa

# Positionals
objects=`ls $data_dir/models/*.ply -v1 | grep -v rhino`
scenes=`ls $data_dir/scenes/*.ply -v1`
pose_dir=$data_dir/GroundTruth_3Dscenes
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --resolution-query=$resq --threshold=$thres --features=$features"
options_rec="--ransac-iterations=$ransac_iter --inlier-fraction=$inlier_frac --icp-iterations=$icp_iter --correspondence-fraction=$corr_frac --translation-tolerance=$trans_tol --rotation-tolerance=$rot_tol"
flags="--pca --pose-file-model-first --verbose --visualize"

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 

#gdb -ex r --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 2> /dev/null
