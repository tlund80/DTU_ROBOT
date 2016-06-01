#!/bin/bash

# Make sure temp dir exists
if [ ! -d tmp ]; then mkdir tmp; fi

# Scripts to run
scripts=("benchmark_bologna1_noise0.1.sh" "benchmark_bologna1_noise0.3.sh" "benchmark_bologna2.sh" "benchmark_uwa.sh" "benchmark_queens.sh" "benchmark_rgbd_scenes_0.125.sh")

# Features to use for fusion
features=ecsad,hist128,rops,shot,si

# Tuned radii per dataset for the chosen features
rad_array=("30,30,30,30,30", "30,30,30,30,30", "30,30,30,30,30", "10,20,12.5,17.5,10", "10,22.5,12.5,20,12.5", "25,25,20,25,22.5")

# Loop over all scripts
for i in `seq 1 ${#scripts[@]}`
do
	# Get zero-based index
	idx=`expr $i - 1`
	
	# Get script file and radii
	script=${scripts[idx]}
	radii=${rad_array[idx]}
	
    # Create a temp script
    script_base=`basename $script .sh`
    script_tmp=`mktemp --dry-run --suffix=.sh --tmpdir=tmp "$script_base"XXX`
    cp $script $script_tmp
    
    echo "---------- Running temp fusion script $script_tmp ----------"
    
    # Replace radius variable value inside temp script
    sed -i 0,/rad=.*/s//rad=$radii/ $script_tmp
    
    # Append '_fusion' to output dir inside temp script
	output_dir_line=`grep output_dir= $script_tmp`
	output_dir_line_new="$output_dir_line"_fusion
	sed -i s@$output_dir_line@$output_dir_line_new@g $script_tmp
    
    # Run inside a detached screen instance (non-blocking)
    screen -d -m sh $script_tmp --features=$features --fusion $*
done
