/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace base_controller_plugins{

  class Omni4 : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  
  private:
  	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  	void TimerCallback(const ros::TimerEvent& event);
  	void CalcWheelSpeed(double actualDt);
  
  	double MaximumAcceleration;
  	double MaximumVelocity;
  	double RobotRadius;
  	double wheel_radius;
  
  	bool InvertX = false;
  	bool InvertY = false;
  	bool InvertZ = false;
  
  	bool LimitVelocity = true;
  	bool LimitAcceleration = true;
  
  	ros::NodeHandle nh;
  	ros::NodeHandle _nh;
  
  	ros::Subscriber cmdVel_sub;
  	ros::Timer control_tim;
  
  	ros::Publisher motor0CmdVel_pub;
  	ros::Publisher motor1CmdVel_pub;
  	ros::Publisher motor2CmdVel_pub;
  	ros::Publisher motor3CmdVel_pub;
  
  	double targetVelX;
  	double targetVelY;
  	double targetRotZ;
  
  	double current_time = 0.0;
  	double last_time = 0.0;
  
  	double lastTarget[4];
  	std_msgs::Float64 motorCmdVelmsg[4];
  
    nav_msgs::Odometry odom_twist;
  	ros::Publisher odom_twist_pub;
  };
  
  void Omni4::onInit(){
    nh = getNodeHandle();
    //constructor
    _nh = getPrivateNodeHandle();
  
  	_nh.param("motor_max_acc", this->MaximumAcceleration, 0.0);
  	_nh.param("motor_max_vel", this->MaximumVelocity, 0.0);
  	_nh.param("robot_radius", this->RobotRadius, 0.258);
  	_nh.param("wheel_radius", this->wheel_radius, 0.0500);
  
  	NODELET_INFO("motor_max_acc : %f", this->MaximumAcceleration);
  	NODELET_INFO("motor_max_vel : %f", this->MaximumVelocity);
  	NODELET_INFO("robot_radius : %f", this->RobotRadius);
  
  	if(this->MaximumVelocity < 0)
  	{
  		this->LimitVelocity = false;
  	}
  
  	if(this->MaximumAcceleration < 0)
  	{
  		this->LimitAcceleration = false;
  	}
  
  	_nh.param("invert_x", this->InvertX, false);
  	_nh.param("invert_y", this->InvertY, false);
  	_nh.param("invert_z", this->InvertZ, false);
  
  	cmdVel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Omni4::CmdVelCallback, this);
  
  	motor0CmdVel_pub = nh.advertise<std_msgs::Float64>("motor0_cmd_vel", 1);
  	motor1CmdVel_pub = nh.advertise<std_msgs::Float64>("motor1_cmd_vel", 1);
  	motor2CmdVel_pub = nh.advertise<std_msgs::Float64>("motor2_cmd_vel", 1);
  	motor3CmdVel_pub = nh.advertise<std_msgs::Float64>("motor3_cmd_vel", 1);
  
  	targetVelX = targetVelY = targetRotZ = 0.0;
  
  	lastTarget[0] = 0.0;
  	lastTarget[1] = 0.0;
  	lastTarget[2] = 0.0;
    lastTarget[3] = 0.0;
  
  	motorCmdVelmsg[0].data = 0.0;
  	motorCmdVelmsg[1].data = 0.0;
  	motorCmdVelmsg[2].data = 0.0;
    motorCmdVelmsg[3].data = 0.0;
  
    odom_twist = nav_msgs::Odometry();
    odom_twist_pub = nh.advertise<nav_msgs::Odometry>("odom_twist", 10);

    //main
  	NODELET_INFO("base_controller node has started.");
  }
  
  void Omni4::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
  	this->targetVelX = static_cast<double>(msg->linear.x);
  	this->targetVelY = static_cast<double>(msg->linear.y);
  	this->targetRotZ = static_cast<double>(msg->angular.z);
  
  	if(this->InvertX)
  	{
  		this->targetVelX *= -1;
  	}
  	if(this->InvertY)
  	{
  		this->targetVelY *= -1;
  	}
  	if(this->InvertZ)
  	{
  		this->targetRotZ *= -1;
  	}
  
  	current_time = ros::Time::now().toSec();
  	CalcWheelSpeed(current_time - last_time);
  	motor0CmdVel_pub.publish(motorCmdVelmsg[0]);
  	motor1CmdVel_pub.publish(motorCmdVelmsg[1]);
  	motor2CmdVel_pub.publish(motorCmdVelmsg[2]);
  	motor3CmdVel_pub.publish(motorCmdVelmsg[3]);
  
  	last_time = current_time;
  
   odom_twist.header.frame_id = "/4omni/odom";
   odom_twist.header.stamp = ros::Time::now();
   odom_twist.child_frame_id = "/4omni/odom_link";
   odom_twist.twist.covariance = {
   0.5, 0, 0, 0, 0, 0,  // covariance on gps_x
   0, 0.5, 0, 0, 0, 0,  // covariance on gps_y
   0, 0, 0.5, 0, 0, 0,  // covariance on gps_z
   0, 0, 0, 0.1, 0, 0,  // large covariance on rot x
   0, 0, 0, 0, 0.1, 0,  // large covariance on rot y
   0, 0, 0, 0, 0, 0.1}; // large covariance on rot z
   odom_twist.twist.twist = *msg;
   odom_twist_pub.publish(odom_twist);
  }
  
  void Omni4::CalcWheelSpeed(double actualDt){
  	double t[4];
  
  	t[0] = -((targetVelX * sin(1 * M_PI / 4))	+ (targetVelY * cos(1 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  	t[1] = -((targetVelX * sin(3 * M_PI / 4))	+ (targetVelY * cos(3 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  	t[2] = -((targetVelX * sin(5 * M_PI / 4))	+ (targetVelY * cos(5 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
    t[3] = -((targetVelX * sin(7 * M_PI / 4))	+ (targetVelY * cos(7 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  
  	double _k = 1.0;
  
  	if(this->LimitVelocity){
  		for(int i = 0; i < 4; i++){
  			auto _a = fabs(t[i]);
  			if(_a * _k > this->MaximumVelocity){
  				_k = this->MaximumVelocity / _a;
                  NODELET_WARN("An infeasible velocity command detected! You might want to look into it.");
  			}
  		}
  
  		for(int i = 0; i < 4; i++){
  			t[i] *= _k;
  		}
  	}
  
  	if(this->LimitAcceleration){
  		float maxVelDelta = this->MaximumAcceleration * actualDt;
  
  		_k = 1.0;
  
  		for(int i = 0; i < 4; i++){
  			double diffabs = fabs(t[i] - lastTarget[i]);
  			if(diffabs * _k > maxVelDelta){
  				_k = maxVelDelta / diffabs;
  				NODELET_WARN("An infeasible acceleration detected! You might want to look into it.");
  			}
  		}
  
  		for(int i = 0; i < 4; i++){
  			t[i] = lastTarget[i] + ((t[i] - lastTarget[i]) * _k);
  		}
  	}
  
  	for(int i = 0; i < 4; i++){
  		this->lastTarget[i] = t[i];
  		this->motorCmdVelmsg[i].data = t[i];
  	}
  }
  
}// namespace base_controller_plugins
PLUGINLIB_EXPORT_CLASS(base_controller_plugins::Omni4, nodelet::Nodelet);
