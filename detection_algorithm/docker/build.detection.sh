#!/bin/bash
cd /home/isilva/3d_detection/detection_algorithm
docker build -t pcdet_pp_ros1_image:v1 . -f docker/Dockerfile.detection
