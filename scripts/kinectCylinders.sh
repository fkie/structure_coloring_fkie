#!/bin/bash
source dataDirs.txt
if [ $# -lt 1 ]
then
    echo "Please specify parameter file"
    exit
fi
params=$1
shift # shift parameters (removing first)
filePrefix="kinect.cylinders"
./segment.sh $KINECT_CYLINDERS $filePrefix 30 $params --cylinder $*

./compare.sh $KINECT_CYLINDERS 29
