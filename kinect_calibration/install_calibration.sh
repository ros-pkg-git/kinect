#! /bin/bash

target_dir=`rospack find kinect_camera`/info
cp $1/calibration_depth.yaml $1/calibration_rgb.yaml $1/kinect_params.yaml $target_dir
echo Wrote calibration files to $target_dir
