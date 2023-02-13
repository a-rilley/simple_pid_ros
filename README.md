# simple_pid_ros
## A Simple PID Controller for Robot Operating System (ROS)
This program was written to provide PID control to robotic control systems and to easily publish and subscribe to ROS topics relevant to the controller. This controller can be used to control multiple processes by running multiple instances (nodes) in the launch file.

## simple_pid.launch
Use this file to:
  - Set proportional gain (kp), integral gain (ki), and derivative gain (kd) values.
  - Set maximum (uMax) and minimum (uMin) control values. Example: Min and max servo position.
  - Set your setpoint timeout if desired. Leave as -1.0 for no timeout/reset.
  - Map your topics without having to edit the .cpp file.
      - Setpoint (SP): Input value / desired value.
      - Process Variable (PV): Control value to compare against desired value.
      - Control Variable (u): Output or computed value.
  - If you wish to run multiple PID controllers, copy/paste node then change the "name" of each node (example: simple_pid1, simple_pid2, etc) and assign the appropriate topics to each node.  

```
<launch>
    <node pkg="simple_pid_ros" type="simple_pid" name="simple_pid" output="screen" >
      <param name="kp" value="<your_p_value>" />
      <param name="ki" value="<your_i_value>" />
      <param name="kd" value="<your_d_value>" />
      <param name="uMax" value="<your_max_output_value>" />
      <param name="uMin" value="<your_min_output_value>" />
      <param name="setpoint_timeout_" value="<your_timeout_value>" />
      <remap from="setpoint_topic_" to="<your_setpoint_topic>" />
      <remap from="process_variable_topic_" to="<your_process_variable_topic>" />
      <remap from="pid_control_topic" to="<your_pid_control_topic>" />
     </node>
</launch>
```
## What is a PID Controller? 
A PID controller is used to provide continuous modulated control in feedback loop control systems. A PID controller continuously calculates the difference between a desired setpoint and a measured process variable as an error value. A correction is then applied to the process/plant based on the sum of proportional, integral, and derivative control values. 


![alt text](https://plcynergy.com/wp-content/uploads/2021/01/PID-controller-1024x329.jpg)

PID:
  - P is for proportional control. Proportional to the difference between the desired setpoint and the process variable ( SP - PV ).
  - I is for integral control. Integrates past values of proportional errors over time.
  - D is for derivative control. Estimates the future values of the proportional error based its current rate of change. 
