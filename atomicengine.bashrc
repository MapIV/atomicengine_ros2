export rosdistro=humble
export rmw_implementation=rmw_cyclonedds_cpp
export base_image=ros:humble-ros-base-jammy
export cuda_version=12.3
export cudnn_version=8.9.5.29-1+cuda12.2
export tensorrt_version=8.6.1.6-1+cuda12.0
export pre_commit_clang_format_version=17.0.5

export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/autoware/.ros/cyclonedds.xml

source /home/autoware/nebula/install/setup.bash
source /home/autoware/atomicengine/install/setup.bash
alias run-ot128='ros2 launch nebula_ros hesai_launch_all_hw.xml sensor_model:=Pandar128E4X'
alias run-rviz='rviz2 -d atomicengine/rviz.rviz'
alias run-atomicengine='ros2 launch atomicengine_launch atomicengine.launch.xml' 
alias run-csv-analysis='python3 /home/autoware/atomicengine/csv_analysis.py'
