#!/bin/bash

if [ $# -lt 2 ]
then
    echo "Not enough parameters for compare"
    exit
fi
dataDir="$1"
nr="$2"

checkNormals=`ls $dataDir/*.0.ms-nor`
if [ -n "$checkNormals" ]
then
    compare=""
else
    compare="-noNormals"
fi

filename=`ls $dataDir/*.0.ms-seg`
filename=`basename $filename`
prefix="${filename%.*}" # strip extension
prefix="${prefix%.*}" # strip number
currentDir=`pwd`
mydir=`dirname $0`
mydir="${currentDir}/${mydir}"

cd $dataDir
$mydir/../bin/compare $prefix -start 0 -end $nr -T 0.8 $compare
