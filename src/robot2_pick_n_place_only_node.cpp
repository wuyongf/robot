// ROS Includes
#include <ros/ros.h>

// User defined includes
#include <robot2_pick_n_place_only/robot2_pick_n_place_only.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot2_pick_n_place_only");
  robot2_pick_n_place_only::PNPOnlyNode pnp;
  ros::spin();

  return 0;
}

