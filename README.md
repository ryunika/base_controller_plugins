base_controller_plugins 
====
[![Build Status](https://travis-ci.org/ryugirou/robot.svg?branch=master)](https://travis-ci.org/ryugirou/robot)
## Usage
launchファイルでの使い方
```xml
  <arg name="manager_name" default="nodelet_manager" />
  <arg name="nodelet_mode" default="load" /><!-- set to standalone if you want to use as node-->
   
  <group if="$(eval nodelet_mode=='load')">
    <node pkg="nodelet" type="nodelet" name="$(arg manager_name)" args="manager" output="screen"/>
  </group>

  <!-- convert x-y velocity to each motor velocity -->
  <node pkg="nodelet" type="nodelet" name="base_controller" args="$(arg nodelet_mode) base_controller_plugins/Omni4 $(arg manager_name)" output="screen">
    <param name="motor_max_acc" value="50" />
    <param name="motor_max_vel" value="80" />
    <param name="invert_x" value="false" />
    <param name="invert_y" value="true" />
    <param name="invert_z" value="true" />
    <param name="robot_radius" value="0.25" />
    <param name="wheel_radius" value="0.1" />
    <remap from="motor0_cmd_vel" to="base/motor0_cmd_vel" />
    <remap from="motor1_cmd_vel" to="base/motor1_cmd_vel" />
    <remap from="motor2_cmd_vel" to="base/motor2_cmd_vel" />
    <remap from="motor3_cmd_vel" to="base/motor3_cmd_vel" />
  </node>    
```
