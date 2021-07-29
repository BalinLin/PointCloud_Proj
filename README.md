# PointCloud_Proj
* About `*.las` and `*.pcd` processing.

* `./script/exper/vbox*.py` process IMU info.
* `./script/exper/LasToPcd.py` process WGS84.las to bin.pcd.


## Usage
``` bash
# Python in ./script
cd ./script

# [Cpp build](https://blog.csdn.net/sweetorange_/article/details/112242852)
mkdir build
cd build
cmake ..
make
cd ..

# produce voxel data
build/VoxelPCD twAll_bin.pcd twDown05_bin.pcd 0.5
```