# simple_pid_ros
## A simple PID Controller for ROS
This program was written to easily publish and subscribe to topics relevant to PID control.

## simple_pid.launch
Use this file to:
  - Set proportional gain (kp), integral gain (ki), and derivative gain (kd) values.
  - Set maximum (uMax) and minimum (uMin) control values. Example: Min and max servo position.
  - Set your setpoint timeout if desired. Leave as -1.0 for no timeout/reset.
  - Map your topics without having to edit the .cpp file.
      - Setpoint: Desired value.
      - Process Variable: Input value to compare against desired value.
      - Control Variable: Output or computed value. 

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
