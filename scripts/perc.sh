#!/bin/bash
source dataDirs.txt
if [ $# -lt 1 ]
then
    echo "Please specify parameter file"
    exit
fi
filePrefix="perc.test"
./segment.sh $PERCEPTRON $filePrefix 30 $*

./compare.sh $PERCEPTRON 29
