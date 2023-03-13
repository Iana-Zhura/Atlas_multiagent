#!/bin/bash 

source /home/iana/anaconda3/bin/activate atlas
# conda activate atlas
python3 ~/multi_agent_ws/src/render/render/scripts/prepare_data.py -s --path DATAROOT --path_meta METAROOT --dataset sample
python3 ~/multi_agent_ws/src/render/render/scripts/inference.py -s --model results/release/semseg/final.ckpt --scenes METAROOT/sample/sample1/info.json --voxel_dim 208 208 80
