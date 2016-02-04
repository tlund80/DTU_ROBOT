#!/bin/sh

if [ "$1" = "" -o ! -d $1 ]
then
    echo "Usage: $0 <input_dir>"
    exit
fi

for ply in $1/*.ply
do
    echo "Formatting $ply..."
    meshlabserver -i $ply -o $ply -om vn 1>/dev/null
done
