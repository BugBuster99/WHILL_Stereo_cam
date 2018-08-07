#!/bin/sh


rosservice call /right_cam/left_master/set_frame_rate 15 &
rosservice call /right_cam/right_slave/set_frame_rate 15 
rosservice call /left_cam/left_master/set_frame_rate 15 &
rosservice call /left_cam/right_slave/set_frame_rate 15 

#rosservice call /right_cam/left_master/set_stream_mode Trigger
#rosservice call /right_cam/right_slave/set_stream_mode Trigger
#rosservice call /left_cam/left_master/set_stream_mode Trigger
#rosservice call /left_cam/right_slave/set_stream_mode Trigger

