#!/bin/bash

TESTS_PATH=$1
BASE_YAML=$2
WORK_PATH=$3
FLOW_EXE=$4
NPROC=$5
TASKS=$WORK_PATH/tasks

rm $TASKS
for f in $(cd $TESTS_PATH; find . -name '*.yaml'); do 
    echo "./single.bash $TESTS_PATH/$f $BASE_YAML $WORK_PATH/$f $FLOW_EXE" >> $TASKS
done

cat $TASKS | parallel --progress -j $NPROC {}
