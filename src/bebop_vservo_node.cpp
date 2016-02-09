#include <ros/ros.h>
#include "bebop_vservo/bebop_vservo_ctrl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_vservo_node");
  ros::NodeHandle nh;
  bebop_vservo::BebopVServoCtrl bebop_vservo_ctrl(nh);

  ROS_INFO("[VSER] Starting bebop_vservo_node ...");
  bebop_vservo_ctrl.Spin();

  return 0;
}
