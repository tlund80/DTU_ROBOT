#!/bin/bash

# Scripts to run
scripts="benchmark_bologna1_noise0.1.sh benchmark_bologna1_noise0.3.sh benchmark_bologna2.sh benchmark_queens.sh benchmark_rgbd_scenes_0.125.sh benchmark_uwa.sh"

# Loop over all scripts, replace radius value and execute
for script in $scripts
do    
    echo "---------- Running script $script ----------"
    # Run inside a detached screen instance (non-blocking)
    screen -d -m sh $script $*
done
