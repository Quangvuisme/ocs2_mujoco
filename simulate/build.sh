#!/bin/bash

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Run cmake and make
cmake .. -DUSE_ROS2=OFF -DENABLE_CUDA=OFF -DUSE_OPENCV=OFF

make -j8
