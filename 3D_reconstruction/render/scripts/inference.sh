#!/bin/bash 

source ~/anaconda3/etc/profile.d/conda.sh
conda activate atlas
python3 -s ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/prepare_data.py --path ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/DATAROOT --path_meta ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/METAROOT --dataset sample
python3 -s ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/inference.py --model ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/results/release/semseg/final.ckpt --scenes ~/anaconda3/Atlas/src/3D_reconstruction/render/scripts/METAROOT/sample/sample1/info.json --voxel_dim 208 208 80
echo $?
