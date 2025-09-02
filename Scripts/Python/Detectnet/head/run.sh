#!/bin/bash

echo "Hello"

cd ~/jetson-inference
docker/run.sh --volume ~/jose:/jose -c dustynv/ros-pyserial -r /jose/r2d2/python/detectnet/head/run_head.sh

