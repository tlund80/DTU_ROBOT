#!/bin/bash

# Make sure temp dir exists
if [ ! -d tmp ]; then mkdir tmp; fi

# Scripts to run
scripts=("benchmark_bologna1_noise0.1.sh" "benchmark_bologna1_noise0.3.sh" "benchmark_bologna2.sh" "benchmark_uwa.sh" "benchmark_queens.sh" "benchmark_rgbd_scenes_0.125.sh")

# Features to use for PCA
features=ecsad,rops,shot,usc

# Tuned radii per dataset for the chosen features
rad_array=("30,30,30,30", "30,30,30,30", "30,30,30,30", "10,12.5,17.5,12.5", "10,12.5,20,12.5", "25,20,25,25")

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
    
    echo "---------- Running temp PCA script $script_tmp ----------"
    
    # Replace radius variable value inside temp script
    sed -i 0,/rad=.*/s//rad=$radii/ $script_tmp
    
    # Append '_pca' to output dir inside temp script
	output_dir_line=`grep output_dir= $script_tmp`
	output_dir_line_new="$output_dir_line"_pca
	sed -i s@$output_dir_line@$output_dir_line_new@g $script_tmp
    
    # Run inside a detached screen instance (non-blocking)
    screen -d -m sh $script_tmp --features=$features --pca $*
done
