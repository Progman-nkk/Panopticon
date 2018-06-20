#include <ros/ros.h>
#include "servo_control/CommandServo.h"
#include "ServoProxy.h"


void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "servo_control_node");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	printf("Servo_Control_Node starting. Press Enter to continue.");
	ServoProxy servoProxyActive(&nh, p_nh);
	ros::ServiceServer service = nh.advertiseService("command_servo", &ServoProxy::processCommand, &servoProxyActive);
	servoProxyActive.gameLoop();
	
	msgUser("Quitting. Press any key to continue.");
	return 0;
}
	