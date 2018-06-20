#ifndef __SERVOPROXY_H_INCLUDED__
#define __SERVOPROXY_H_INCLUDED__

#include "pubSysCls.h"	
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include "servo_control/CommandServo.h"

class ServoProxy{
	public:		
		ServoProxy(ros::NodeHandle* nh, ros::NodeHandle p_nh);	
		void gameLoop();
		bool processCommand(servo_control::CommandServo::Request &req,
             servo_control::CommandServo::Response &res);

	private:
		void update(double rotationAroundZ, ros::Time rotationTimeStamp);		
		bool readyNode();
		bool closeNode();
		void startMotion();
		void stopMotion();

        ros::Publisher pub_;
		ros::NodeHandle nh;
		ros::NodeHandle p_nh;
		ros::Timer updateTimer;
		int rate;
		tf::TransformBroadcaster br;
  		tf::Transform transform;
		//sFoundation
		sFnd::SysManager myMgr;
		std::vector<std::string> comHubPorts;
		double encoded_revolutions;
		int encoder_resolution;
		int vel_lim_rpm;
		int acc_lim_rpm_per_sec;

        ros::Time startOfRevolution;
        bool startSwitch;
        ros::Time endOfRevolution;
        bool endSwitch;
};

#endif