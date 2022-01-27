#!/bin/bash

roslaunch realsense2_camera rs_camera.launch align_depth:=true & roslaunch pose color_and_depth.launch 
