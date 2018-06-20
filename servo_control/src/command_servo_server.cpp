#include "ros/ros.h"
#include "servo_control/CommandServo.h"

bool respond(servo_control::CommandServo::Request &req,
             servo_control::CommandServo::Response &res){
                 switch(req.command){
                    case 'g':   
                        res.response_code = 1;
                        break;
                    case 's':      
                        res.response_code = 0;
                        break;  
                    case 'v':     
                        res.response_code = req.modifier;
                        break;
                    case 'a':     
                        res.response_code = req.modifier;
                        break;
                 }
                 return true;
             }

int main(int argc, char **argv){
    ros::init(argc, argv, "command_servo_server");
    ros:: NodeHandle n;

    ros::ServiceServer service = n.advertiseService("command_servo", respond);
    ROS_INFO("Ready to respond.");

    ros::spin();
    return 0;
}