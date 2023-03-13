#!/bin/bash 

source ~/anaconda3/etc/profile.d/conda.sh
conda activate atlas
python3 -s ~/anaconda3/Atlas/src/render/scripts/prepare_data.py --path ~/anaconda3/Atlas/src/render/scripts/DATAROOT --path_meta ~/anaconda3/Atlas/src/render/scripts/METAROOT --dataset sample
python3 -s ~/anaconda3/Atlas/src/render/scripts/inference.py --model ~/anaconda3/Atlas/src/render/scripts/results/release/semseg/final.ckpt --scenes ~/anaconda3/Atlas/src/render/scripts/METAROOT/sample/sample1/info.json --voxel_dim 248 248 96
