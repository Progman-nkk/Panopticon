#include "ServoProxy.h"


#define TIME_TILL_TIMEOUT	10000
#define PI 3.1415926535897


using namespace sFnd;

ServoProxy::ServoProxy(ros::NodeHandle* _nh, ros::NodeHandle _p_nh):nh(*_nh){
	// Get/Set Parameters
	p_nh = _p_nh;
    pub_ = nh.advertise<std_msgs::Time> ("sweep_trigger", 5);

	double raw_revolutions;
	encoder_resolution = 6400;
	p_nh.getParam("revolutions", raw_revolutions);
	encoded_revolutions = raw_revolutions*encoder_resolution;
	p_nh.getParam("acc_lim_rpm_per_sec", acc_lim_rpm_per_sec);
	p_nh.getParam("vel_lim_rpm", vel_lim_rpm);

	printf("Got vel: %d", vel_lim_rpm);
	printf("Got acc: %d", acc_lim_rpm_per_sec);
	printf("Got enc_revs: %f", encoded_revolutions);
	printf("Got parameters\n");

	readyNode();
}
bool ServoProxy::readyNode(){
	try
	{ 
		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", (int)comHubPorts.size());
		myMgr.ComHubPort(0, comHubPorts[0].c_str());
		myMgr.PortsOpen(1);

		IPort &myPort = myMgr.Ports(0);
		printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
		
		INode &theNode = myPort.Nodes(0);

		theNode.EnableReq(false);
		myMgr.Delay(200);
		theNode.Status.AlertsClear();
		theNode.Motion.NodeStopClear();
		theNode.EnableReq(true);

		double timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
		while (!theNode.Motion.IsReady()) {
					if (myMgr.TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for Node to enable\n");
						printf("Press any key to continue.");
						return false;
					}
				}
		printf("Node is enabled.\n");
		theNode.Motion.MoveWentDone();
		theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
		theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
		theNode.Motion.AccLimit = acc_lim_rpm_per_sec;		//Set Acceleration Limit (RPM/Sec)
		theNode.Motion.VelLimit = vel_lim_rpm;				//Set Velocity Limit (RPM)
		return true;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to ready Nodes n\n");
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		printf("Press any key to continue.");
		return false;
	}
}
bool ServoProxy::processCommand(servo_control::CommandServo::Request &req,
             servo_control::CommandServo::Response &res){
				 printf("processing command\n");
                 switch(req.command){
                    case 'g':   
						startMotion();
                        res.response_code = 1;
						
                        break;
                    case 's':     
						stopMotion(); 
                        res.response_code = 0;
                        break;  
                    case 'v':   
						// INode &theNode = myMgr.Ports(0).Nodes(0);
						// theNode.Motion.VelLimit = req.modifier;
						// p_nh.setParam("acc_lim_rpm_per_sec", req.modifier);
                        res.response_code = req.modifier;
                        break;
                    case 'a':    
						// INode &theNode = myMgr.Ports(0).Nodes(0);
						// theNode.Motion.AccLimit = req.modifier;
						// p_nh.setParam("vel_lim_rpm", req.modifier);
                        res.response_code = req.modifier;
                        break;
                 }
                 return true;
}
void ServoProxy::update(double rotationAroundZ, ros::Time rotationTimeStamp){
    std_msgs::Time _time;
    _time.data.sec = 4;
    _time.data.nsec = 12;
    if(rotationAroundZ >= 179.925 && startSwitch){
        printf("Start Switch: %lf \n", rotationAroundZ);
        startOfRevolution = rotationTimeStamp;
        startSwitch = false;
        endSwitch = true;
		_time.data = rotationTimeStamp;
        pub_.publish(_time);
        printf("TimeNow: %f \n", rotationTimeStamp.toSec());
    }
    if(rotationAroundZ <= 0.25 && endSwitch){
        printf("End Switch: %lf \n", rotationAroundZ);
        endOfRevolution = rotationTimeStamp;
        startSwitch = true;
        endSwitch = false;
		_time.data = rotationTimeStamp;
        pub_.publish(_time);
    }
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    tf::Quaternion q;
    q.setRPY(0.0, (-90*PI/180), (rotationAroundZ*PI/180));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, rotationTimeStamp, "base_link", "base_laser"));
}
bool ServoProxy::closeNode(){
	INode &theNode = myMgr.Ports(0).Nodes(0);
	printf("Disabling the node\n");
	theNode.EnableReq(false);
	myMgr.PortsClose();

	return true;
}
void ServoProxy::stopMotion(){
	INode &theNode = myMgr.Ports(0).Nodes(0);
	printf("Stopping the node\n");
	theNode.Motion.NodeStop(STOP_TYPE_RAMP);
}
void ServoProxy::startMotion(){
	INode &theNode = myMgr.Ports(0).Nodes(0);
	printf("Moving the node\n");
	theNode.Motion.MoveVelStart(vel_lim_rpm);
}
void ServoProxy::gameLoop(){
	ros::Rate loop_rate(500);
	printf("running \n");
	INode &theNode = myMgr.Ports(0).Nodes(0);
	while(ros::ok()){		
		theNode.Motion.PosnMeasured.Refresh();
		double currentPosition = theNode.Motion.PosnMeasured.Value();
		ros::Time t = ros::Time::now();
		int a = currentPosition/6400;
		double b = currentPosition - (a*6400);
		double c = (b/6400)*360;
		update(c, t);

		ros::spinOnce();
		loop_rate.sleep();
	}
	closeNode();
}