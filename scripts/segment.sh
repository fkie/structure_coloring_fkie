#!/bin/bash

if [ $# -lt 3 ]
then
    echo "Not enough parameters"
    exit
fi

dataDir="$1"
filePrefix="$2"
nr="$3"
params=
runtime=
if [ -n "$4" ]; then
    params="-p $4"
    runtime=`basename $4`
    runtime="${runtime%.*}.time" # strip extension
    if [ -e $runtime ]
    then
        rm $runtime
    fi
    runtime="--runtimeFile ${runtime}"
fi

if [ -e $dataDir/$filePrefix.0.range ]
then
    inputType="range"
elif [ -e $dataDir/$filePrefix.0.pcd ]; then
    inputType="pcd"
else
    inputType="perc"
fi

i=0
while [ $i -lt $nr ]
do
    if [ "$inputType" = "range" ]
    then
        input="--prefix $dataDir/$filePrefix.$i"
    elif [ "$inputType" = "pcd" ]; then
        input="--pcd $dataDir/$filePrefix.$i.pcd"
    else
        input="--percPrefix $dataDir/$filePrefix.$i"
    fi
#    echo ../bin/segmentSC $input -m $dataDir/$filePrefix.$i $runtime $params $5 $6 $7 $8 $9
    ../bin/segmentSC $input -m $dataDir/$filePrefix.$i $runtime $params $5 $6 $7 $8 $9
    i=`expr $i + 1`
#    read -p "Press Enter to continue..."
done

