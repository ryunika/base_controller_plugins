/*
 * vel_control.cpp
 *
 *  Created on: Nov 18, 2019
 *      Author: ryu
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace base_controller_plugins{
  class Vel_control : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
	  ros::NodeHandle nh;
  	ros::NodeHandle nh_priv;
		std_msgs::Float64 position;
		ros::Publisher pos_pub;
		ros::Subscriber vel_sub;
		ros::Timer control_tim;
		double limit[2];
		double vel=0;
		double ctrl_freq;
  	void TimerCallback(const ros::TimerEvent& event);
		void VelCallback(const std_msgs::Float64::ConstPtr& msg); 
  };

	void Vel_control::onInit(){
		nh = getNodeHandle();
    nh_priv = getPrivateNodeHandle();
		position.data = 0;
		if (!nh_priv.getParam("ctrl_freq", ctrl_freq))
  	{
    	ctrl_freq = 500;
  	}
		if(!nh_priv.getParam("lower_limit", limit[0]))
    {
      limit[0] = 0;
    }
		if(!nh_priv.getParam("upper_limit", limit[1]))
    {
      limit[1] = 6.28;
    }
		pos_pub = nh.advertise<std_msgs::Float64>("pos", 1);
		vel_sub	= nh.subscribe<std_msgs::Float64>("vel", 1000, &Vel_control::VelCallback, this);
		control_tim = nh.createTimer(ros::Duration(1/ctrl_freq), &Vel_control::TimerCallback, this);
	}

	void Vel_control::TimerCallback(const ros::TimerEvent& event){
		position.data += vel/ctrl_freq;
		if(vel > 0 && position.data > limit[1]){
			position.data = limit[1];
		} 
		else if(vel < 0 && position.data < limit[0]){
			position.data = limit[0];
		}
		pos_pub.publish(position);
	}

	void Vel_control::VelCallback(const std_msgs::Float64::ConstPtr& msg){
		vel=msg->data;
	}
}// namespace base_controller_plugins
PLUGINLIB_EXPORT_CLASS(base_controller_plugins::Vel_control, nodelet::Nodelet);