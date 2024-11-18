# Atomic Engine ROS2

## Installation
* Setup PC with MapIV Ubuntu 22.04 ISO.
* Install CUDNN and TensorRT following [these instructions](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/tensorrt#manual-installation)
* `rosdep install -y --from-paths src --ignore-src`
* `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

Everything should build with after rosdep, but you probably need to manually install:
* `sh additional-install.sh`

`atomicengine_data` is in [Google Drive](https://drive.google.com/drive/u/1/folders/1mJbbJsvNGGX3z_wKhysG9Xm5eZ4HYekd).
