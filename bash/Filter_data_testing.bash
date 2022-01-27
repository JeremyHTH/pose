#!/bin/bash

rqt_plot /left_Angle /left_Angle_smooth & rqt_plot /right_Angle /right_Angle_smooth & roslaunch pose Filter_data_testing.launch 
