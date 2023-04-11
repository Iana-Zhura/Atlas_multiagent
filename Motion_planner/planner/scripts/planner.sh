#!/bin/bash 

source ~/anaconda3/etc/profile.d/conda.sh
conda activate planner
python3.9 ~/anaconda3/Atlas/src/Motion_planner/planner/scripts/scripts/run_planner.py

echo $?
