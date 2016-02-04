#!/bin/sh

if [ "$1" = "" -o ! -d "$1" -o "$2" = "" -o ! -f "$2" ]
then
    echo "Usage: $0 <input_dir> <meshlab_script>"
    exit
fi

for ply in $1/*.ply
do
    echo "Decimating $ply..."
    meshlabserver -i $ply -o $ply -s $2 -om vn 1>/dev/null 2>/dev/null
done
