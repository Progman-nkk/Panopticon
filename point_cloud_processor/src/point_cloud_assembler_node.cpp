#include "ros/ros.h"
#include <cstdlib>
#include "laser_assembler/AssembleScans.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Time.h"


namespace laser_assembler
{

  class PeriodicSnapshotter
  {
    public:
      PeriodicSnapshotter(ros::NodeHandle* nh_);
      void timerCallback(const ros::TimerEvent& e);
      void subCallback(const std_msgs::Time::ConstPtr& message_holder);
    private:
      ros::NodeHandle nh_;
      ros::Publisher pub_;
      ros::Subscriber sub_;
      ros::ServiceClient client_;
      ros::Timer timer_;
      bool first_time_;

      ros::Time lastRevolution;
      ros::Time currentRevolution;



  }; // close class
    PeriodicSnapshotter::PeriodicSnapshotter(ros::NodeHandle* nodeHandle):nh_(*nodeHandle) 
    {
      // Create a publisher for the clouds that we assemble
      pub_ = nh_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

      sub_ = nh_.subscribe("sweep_trigger", 1, &PeriodicSnapshotter::subCallback, this);

      // Create the service client for calling the assembler
      client_ = nh_.serviceClient<AssembleScans>("assemble_scans");

      // Start the timer that will trigger the processing loop (timerCallback)
      //timer_ = nh_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);

      // Need to track if we've called the timerCallback at least once
      first_time_ = true;
      ROS_INFO("spinning...");
      ros::spin();
    }
    void PeriodicSnapshotter::subCallback(const std_msgs::Time::ConstPtr& message_holder){
      if (first_time_)
      {
          first_time_ = false;
          lastRevolution = message_holder->data;
          ROS_INFO("first time");
          return;
      }
      lastRevolution = currentRevolution;
      currentRevolution = message_holder->data;
      ROS_INFO("lastRevolution: %f", lastRevolution.toSec());
      ROS_INFO("currentRevolution: %f", currentRevolution.toSec());
      // Populate our service request based on our timer callback times
      AssembleScans srv;
      srv.request.begin = lastRevolution;
      srv.request.end   = currentRevolution;

      // Make the service call
      if (client_.call(srv))
      {
        ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
        pub_.publish(srv.response.cloud);
      }
      else
      {
        ROS_ERROR("Error making service call\n") ;
      }
    }
    void PeriodicSnapshotter::timerCallback(const ros::TimerEvent& e)
    {

      // We don't want to build a cloud the first callback, since we we
      //   don't have a start and end time yet
      if (first_time_)
      {
        first_time_ = false;
        return;
      }

      // Populate our service request based on our timer callback times
      AssembleScans srv;
      srv.request.begin = e.last_real;
      srv.request.end   = e.current_real;

      // Make the service call
      if (client_.call(srv))
      {
        ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
        pub_.publish(srv.response.cloud);
      }
      else
      {
        ROS_ERROR("Error making service call\n") ;
      }
    }
} // close namespace

using namespace laser_assembler;

int main(int argc, char **argv){
    ros::init(argc, argv, "point_cloud_assembler_node");
    ros:: NodeHandle n;;
    ROS_INFO("Waiting for [build_cloud] to be advertised");
    ros::service::waitForService("build_cloud");
    ROS_INFO("Found build_cloud! Starting the snapshotter");
    PeriodicSnapshotter snapshotter(&n);
    
    return 0;
}