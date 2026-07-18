#!/bin/bash
cd /home/isilva/3d_detection/pointclouds_and_bridge
docker build -t publisher_bridge . -f docker/Dockerfile.bridge
