#include "simple_pid.h"

SimplePID::SimplePID()
{
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ROS_INFO("Controller running...");

  // Initialize private node parameters
  private_nh.param<double>("kp", kp_, 1.0);
  private_nh.param<double>("ki", ki_, 1.0);
  private_nh.param<double>("kd", kd_, 1.0);
  private_nh.param<double>("uMax", uMax_, 300000);
  private_nh.param<double>("uMin", uMin_, -300000);
  private_nh.param<double>("setpoint_timeout", setpoint_timeout_, -1.0);
    
  control_pub_ = n.advertise<std_msgs::Float64>("pid_control_topic", 1);

  sp_sub_ = n.subscribe("setpoint_topic_", 1, &SimplePID::setpoint_callback, this);
  pv_sub_ = n.subscribe("process_variable_topic_", 1, &SimplePID::processVariable_callback, this);
  
  private_nh.getParam("kp", kp_);
  private_nh.getParam("ki", ki_);
  private_nh.getParam("kd", kd_);
  private_nh.getParam("uMax", uMax_);
  private_nh.getParam("uMin", uMin_);
  private_nh.getParam("setpoint_timeout", setpoint_timeout_);

  while (ros::ok())
  {
    pidCompute(setpoint, processVariable);
    
    ros::spinOnce();

    ros::Duration(0.001).sleep();
  }

}


SimplePID::~SimplePID()
{

}


void SimplePID::setpoint_callback(const std_msgs::Float64& setpoint_msg)
{
  setpoint = setpoint_msg.data;
  last_setpoint_msg_time_ = ros::Time::now();

}


void SimplePID::processVariable_callback(const std_msgs::Float64& process_variable_msg)
{
  processVariable = process_variable_msg.data;
}


void SimplePID::pidCompute(double setpoint_, double processVariable_) 
{
  double error, u, up, ui, ud;
  ros::Duration dt;
  
  dt = ros::Time::now() - prevTime_;
  prevTime_ = ros::Time::now();

  if (dt.toSec() == 0)
  {
    return;  // Skip this loop to avoid dividing by zero
  }
  
  // Calculate the error (SP-PV)
  error = setpoint_ - processVariable_;

  up = kp_ * error;                             // Proportional error
  ui = ki_ * (ui_old_ + error * dt.toSec());    // Integral error
  ud = kd_ * (error - error_old_) / dt.toSec(); // Derivative error

  u = up + ui + ud;                             // Control variable
  
  // Saturation and anti-wind up
  if (u > uMax_) 
  {
    u = uMax_;
  }
  else if (u < uMin_) 
  {
    u = uMin_;
  }
  else
  {
    ui_old_ = ui; // Integral error is stored if not saturated
  }

  prevTime_ = ros::Time::now();
  error_old_ = error;

  
  control_msg_.data = u;
  control_pub_.publish(control_msg_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_pid");

  SimplePID _pid;

  return 0;
}