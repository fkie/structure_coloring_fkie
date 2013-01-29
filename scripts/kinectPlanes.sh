#!/bin/bash
source dataDirs.txt
if [ $# -lt 1 ]
then
    echo "Please specify parameter file"
    exit
fi
filePrefix="kinect.planes30"
./segment.sh $KINECT_PLANES $filePrefix 30 $*

./compare.sh $KINECT_PLANES 29
