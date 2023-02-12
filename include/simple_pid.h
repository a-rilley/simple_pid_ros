#ifndef SIMPLE_PID_H
#define SIMPLE_PID_H

#include "ros/ros.h"
#include "ros/time.h"
#include "iostream"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"


class SimplePID
{
public:
  // Constructor
  SimplePID();
  
  // Destructor
  ~SimplePID();
  
  // PID enable callback
  void setpoint_callback(const std_msgs::Float64& setpoint_msg);
  void processVariable_callback(const std_msgs::Float64& process_variable_msg);
  
  // Compute PID output
  void pidCompute(double setpoint, double processVariable);


  protected:

    double kp_ = 0;                     // Proportional gain
    double ki_ = 0;                     // Integral gain
    double kd_ = 0;                     // Derivative gain
    double ui_old_ = 0;                 // The value of the integral term from the previous step.
    double error_old_ = 0;              // Error value from the previous step.
    double uMax_ = 0;                   // Upper limit of control variable
    double uMin_ = 0;                   // Lower limit of control variable
    ros::Time prevTime_;                // Time of the previous step
    ros::Time last_setpoint_msg_time_;  // Time of last setpoint message
    double setpoint = 0;                // Setpoint
    double processVariable = 0;         // Process variable
    double setpoint_timeout_ = -1;      // Timeout for setpoint timer
    ros::Publisher control_pub_;        // Publisher for control topic (u)
    ros::Subscriber sp_sub_;            // 
    ros::Subscriber pv_sub_;            // 
    std_msgs::Float64 control_msg_;     // PID output message
    std_msgs::Float64 setpoint_topic_;        // Setpoint string
    std_msgs::Float64 process_variable_topic_;// Process variable string
};


#endif
