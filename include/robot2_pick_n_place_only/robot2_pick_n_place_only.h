#ifndef ROBOT2_PICK_N_PLACE_ONLY_H
#define ROBOT2_PICK_N_PLACE_ONLY_H

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// User defined includes
#include <list>
#include <iterator>
#include <string>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <control_msgs/JointControllerState.h>

#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelStates.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>


namespace robot2_pick_n_place_only
{

class Job
{
  public:
    bool start;
    bool done;

    void reset()
    {
      done = false;
    }

    Job()
    {
      start = false;
      done = false; 
    }
  
};

class PNPOnlyNode
{
  
public:
  Job go_dest_;
  Job lifting_;
  Job gripping_;
  Job lift_to_max_;
  Job go_base_;
  Job lift_to_place_;
  Job open_grip_;
  Job lift_to_max_2_;
  Job go_backward_;
  Job check_in_place_;

  std::list<Job> job_list_;

  PNPOnlyNode();
  ~PNPOnlyNode();


private:
  // handle
  ros::NodeHandle nh_;
  ros::Rate loop_rate{10};

  // publisher
  ros::Publisher ud_mount_pub_;
  ros::Publisher l_hand_pub_;
  ros::Publisher r_hand_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Publisher destination_pub_;
  ros::Publisher twist_pub_;
  
  // subscriber
  ros::Subscriber models_sub_;
  ros::Subscriber ud_mount_state_sub_;
  ros::Subscriber l_hand_state_sub_;
  ros::Subscriber r_hand_state_sub_;
  ros::Subscriber detected_obj_sub_;
  ros::Subscriber move_base_result_sub_;
  ros::Subscriber current_pose_sub_;

  // variables
  std::string tf_prefix_;
  std::string topic_prefix_;
  tf::TransformListener listener;

  gazebo_msgs::ModelStates::ConstPtr models_msg_ = nullptr;

  control_msgs::JointControllerState::ConstPtr ud_mount_state_msg_ = nullptr;
  control_msgs::JointControllerState::ConstPtr l_hand_state_msg_ = nullptr;
  control_msgs::JointControllerState::ConstPtr r_hand_state_msg_ = nullptr;
  move_base_msgs::MoveBaseActionResult::ConstPtr goal_result_msg_ = nullptr;

  geometry_msgs::PoseStamped object_map_pose_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose prev_pose_;


  const std::string object_left_[5] = {"cylinder5", "cylinder4", "cylinder3", "cylinder2", "cylinder1"};
  const double object_radius_[5] = {0.25, 0.2125, 0.175, 0.1375, 0.1};
  const double object_height_ = 0.2;

  const double gripper_min_height_ = 0.0;
  const double gripper_max_height_ = 1.2;
  double gripper_max_width_;


  const double base_x_ = 0.6;
  const double base_y_ = 0.74;
  const double base_safety_offset_ = 0.75;

  const double max_x_ = 4.115;
  const double min_x_ = -4.045;
  const double max_y_ = 5.245;
  const double min_y_ = -5.194;
  const double boundary_safety_offset_ = 0.75;

  double object_tolerance_;
  double base_tolerance_;


  double object_v_pos_;
  double l_prev_value_ = 0.0;
  bool l_hand_stopped_ = false;
  int l_hand_count_ = 0;

  bool goal_published_ = false;
  int base_counter_ = 0;

  int current_object_idx_ = 0;
  int target_;

  bool place_check_[5] = {false,false,false,false,false};
  bool job_cancel_ = false; 
  bool finished_ = false;

  
  // callbacks
  void models_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
  void ud_mount_state_callback(const control_msgs::JointControllerState::ConstPtr& msg);
  void l_hand_state_callback(const control_msgs::JointControllerState::ConstPtr& msg);
  void r_hand_state_callback(const control_msgs::JointControllerState::ConstPtr& msg);
  void move_base_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void current_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void mainloop();
  void gripper_lift(double ud_cmd);
  void gripper_grip(double L_cmd, double R_cmd);
  void go_destination(geometry_msgs::PoseStamped target_pose, double target_tolerance);
  void tf_odom_to_map(geometry_msgs::PoseStamped& original_pose, geometry_msgs::PoseStamped& target_map_pose);
  void go_backward();
  void stop();

};
}  

#endif  
