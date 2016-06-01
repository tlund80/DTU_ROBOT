#!/bin/bash

# Test parameters
dec=-1 # Decimation
rad=25,17.5,25,20,22.5,25,25 # Feature radius multiplier
res=5 # Feature resolution multiplier
thres=1 # Inlier threshold multipler, set to <= 0 for using 1 x mesh resolution

# Paths
exec_dir=build
exec_name=feature_matching_dataset
data_dir=~/workspace/datasets/rgbd
output_dir=output/rgbd_scenes_0.125

# Positionals
queries=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 1'`) # Odd indices
targets=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 0'`) # Even
#queries=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 1' | grep table_1_31`) # Odd indices
#targets=(`ls -v $data_dir/scenes_0.125/*.ply | awk 'NR % 2 == 0' | grep table_1_36`) # Even
pose_dir=$data_dir/ground-truth
pose_separator="-"
pose_suffix=xf

# Options and flags
options="--pose-separator=$pose_separator --decimation=$dec --radius=$rad --resolution=$res --threshold=$thres --metrics=L2_RATIO"
flags="--pose-file-model-first --append --remove-non-overlap --verbose"

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
	
	$exec_dir/$exec_name $query $target $pose_dir $pose_suffix $output_dir $options $flags $* 2> /dev/null
	
	#gdb -ex run --args $exec_dir/$exec_name $query $target $pose_dir $pose_suffix $output_dir $options $flags $*
	
	stat=$?
	
    IFS=$'\n'
    
    if [ $stat -ne 0 ]; then echo "Wrong return code! Aborting..."; exit 1; fi
done

# Restore IFS
IFS=$ifsbak
