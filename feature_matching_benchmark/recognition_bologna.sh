#!/bin/bash

########################################################################
# Test script for the UWA dataset, all models are in [mm] in this case #
########################################################################

# General
dec=0.05 #0.5 #0.125 # Decimation

# Features
features=ecsad,fpfh,ndhist,rops,shot,si,usc,pfh,3dsc
rad=10,10,20,12.5,17.5,10,10,10,10
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
#data_dir=/media/thso/Elements    
data_dir=~/workspace/datasets/bologna
output_dir=output/bologna2

# Positionals
#objects=`ls $data_dir/Object_pose_dataset/models/3D_models/*.ply -v1` 
objects=`ls $data_dir/dataset1-2_models/3D_models/Stanford/*.ply -v1`
#scenes=`ls $data_dir/scene_000/stl/mesh/*.ply -v1` 
scenes=`ls $data_dir/dataset2_scenes/3D_models/Stanford/Random/*.ply -v1`
#scenes=`ls $data_dir/dataset2_scenes/3D_models/Stanford/Random/*.ply -v1 | grep 'Scene0_1-8.ply'` # Fast test of first scene only
pose_dir=$data_dir/dataset2_scenes/3D_models/Stanford/Random
#pose_dir=$data_dir/dataset2_scenes/3D_models/Stanford
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --resolution-query=$resq --threshold=$thres --features=$features"
options_rec="--ransac-iterations=$ransac_iter --inlier-fraction=$inlier_frac --icp-iterations=$icp_iter --correspondence-fraction=$corr_frac --translation-tolerance=$trans_tol --rotation-tolerance=$rot_tol"
flags="--pca --verbose"

# Start
$exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 

#gdb -ex r --args $exec_dir/$exec_name "$objects" "$scenes" $pose_dir $pose_suffix $output_dir $options $options_rec $flags $* 2> /dev/null
