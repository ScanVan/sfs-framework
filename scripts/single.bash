#!/bin/bash

IN_YAML=$1
BASE_YAML=$2
WORK_PATH=$3
FLOW_EXE=$(pwd)/$4

FULL_YAML=$WORK_PATH/full.yaml

mkdir -p $WORK_PATH/dev/log/debug
mkdir -p $WORK_PATH/dev/log/dev

yaml-merge $BASE_YAML $IN_YAML > $FULL_YAML


cd $WORK_PATH
echo $(pwd)
$FLOW_EXE full.yaml
