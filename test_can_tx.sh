#!/bin/bash
for i in {0..18}
do
   ros2 topic pub --once /servo_cmd can_raw_interfaces/msg/ServoCmd "{command_id: $i,servo_id: 66,angle: 22.0,speed: 230.0,mode: 24,torque: 25,duration: 26}" >> test_can_tx.txt
done
