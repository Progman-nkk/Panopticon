#include "ros/ros.h"
#include "servo_control/CommandServo.h"
#include <cstdlib>


int main(int argc, char **argv){
    ros::init(argc, argv, "stack_controller_node");

    ros:: NodeHandle n;

    ros::ServiceClient client = n.serviceClient<servo_control::CommandServo>("command_servo");
    servo_control::CommandServo srv;
    char command_in;
    int modifier_in;
    while(ros::ok()){
        std::cout << "Command:\n" << std::endl;
        std::cin >> command_in;
        std::cout << "Modifier:\n" << std::endl;
        std::cin >> modifier_in;
        
        srv.request.command = command_in;
        srv.request.modifier = modifier_in;
        if(client.call(srv)){
            ROS_INFO("Response: %d", (int)srv.response.response_code);
        }else{
        ROS_INFO("Failed to call");
        return 1;
        }
    }
    return 0;
}