# PointCloud_Proj
* About `*.las` and `*.pcd` processing.

* `./script/vbox*.py` process IMU info.


## Usage
``` bash
# Python in ./script
cd ./script

# [Cpp](https://blog.csdn.net/sweetorange_/article/details/112242852)
mkdir build
cd build
cmake ..
make
cd ..
build/VoxelPCD twAll_bin.pcd twDown05_bin.pcd 0.5
```