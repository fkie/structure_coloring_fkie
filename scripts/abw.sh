#!/bin/bash
source dataDirs.txt
if [ $# -lt 1 ]
then
    echo "Please specify parameter file"
    exit
fi
filePrefix="abw.test"
./segment.sh $ABW $filePrefix 30 $*

./compare.sh $ABW 29
