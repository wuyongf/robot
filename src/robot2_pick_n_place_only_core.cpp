#include <robot2_pick_n_place_only/robot2_pick_n_place_only.h>

namespace robot2_pick_n_place_only
{
    // Constructor
    PNPOnlyNode::PNPOnlyNode()
        : nh_("~") 
    {
        initForROS();
    }

    PNPOnlyNode::~PNPOnlyNode()
    {}

    void PNPOnlyNode::initForROS()
    {
        nh_.param<std::string>("topic_prefix", topic_prefix_, "robot2");
        nh_.param<std::string>("tf_prefix", tf_prefix_, "robot2");
        nh_.param("gripper_max_width", gripper_max_width_, 1.08);
        nh_.param("object_tolerance", object_tolerance_, 0.70);
        nh_.param("base_tolerance", base_tolerance_, 0.65);

        models_sub_ = nh_.subscribe("/gazebo/model_states", 1, &PNPOnlyNode::models_callback, this);

        ud_mount_state_sub_ = nh_.subscribe(topic_prefix_ + "/ud_gripper_mount_controller/state", 1, &PNPOnlyNode::ud_mount_state_callback, this);
        l_hand_state_sub_ = nh_.subscribe(topic_prefix_ + "/gripper_L_controller/state", 1, &PNPOnlyNode::l_hand_state_callback, this);
        r_hand_state_sub_ = nh_.subscribe(topic_prefix_ + "/gripper_R_controller/state", 1, &PNPOnlyNode::r_hand_state_callback, this);

        move_base_result_sub_ = nh_.subscribe(topic_prefix_ + "/move_base/result", 1, &PNPOnlyNode::move_base_result_callback, this);

        current_pose_sub_ = nh_.subscribe(topic_prefix_ + "/odom", 1, &PNPOnlyNode::current_pose_callback, this);

        ud_mount_pub_ = nh_.advertise<std_msgs::Float64>(topic_prefix_ + "/ud_gripper_mount_controller/command", 1);
        l_hand_pub_ = nh_.advertise<std_msgs::Float64>(topic_prefix_ + "/gripper_L_controller/command", 1);
        r_hand_pub_ = nh_.advertise<std_msgs::Float64>(topic_prefix_ + "/gripper_R_controller/command", 1);

        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_prefix_ + "/initialpose", 1);

        destination_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_prefix_ + "/move_base_simple/goal", 1);

        twist_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_prefix_ + "/cmd_vel", 1);
        
        mainloop();
    }

    void PNPOnlyNode::models_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        models_msg_ = msg;

        for (int i = 0; i < models_msg_->name.size(); i++)
        {
            if (models_msg_->name[i] == object_left_[current_object_idx_])
            {
                target_ = i;
                geometry_msgs::PoseStamped object_pose;
                object_pose.header.frame_id = tf_prefix_ + "/odom";
                object_pose.pose = models_msg_->pose[i];

                tf_odom_to_map(object_pose, object_map_pose_);
                go_dest_.start = true;
                break;
            }
        }

    }

    void PNPOnlyNode::tf_odom_to_map(geometry_msgs::PoseStamped& original_pose, geometry_msgs::PoseStamped& target_map_pose)
    {
        try 
        {
            listener.transformPose("map", original_pose, target_map_pose);
        } 
        catch(tf::TransformException& ex) 
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }

    void PNPOnlyNode::go_destination(geometry_msgs::PoseStamped target_pose, double target_tolerance)
    {
        double robot_pos_x = current_pose_.position.x;
        double robot_pos_y = current_pose_.position.y;

        double target_pos_x = target_pose.pose.position.x;
        double target_pos_y = target_pose.pose.position.y;
         
        double target_robot_slop = (target_pos_y - robot_pos_y) / (target_pos_x - robot_pos_x);
        double target_robot_angle = atan(target_robot_slop);

        double goal_x, goal_y, yaw_angle;

        if (target_pos_x >= max_x_ - 2.0 || (target_pos_x < base_x_ - 0.25 && target_pos_x > min_x_ + 2.0 && target_pos_y >= base_y_ - 1.0 && target_pos_y <= base_y_ + 1.0))
        {
            // up
            yaw_angle = 0;
            goal_x = target_pos_x - target_tolerance;
            goal_y = target_pos_y;

        }
        else if (target_pos_x <= min_x_ + 2.0 || (target_pos_x > base_x_ + 0.25 && target_pos_x < max_x_ - 2.0 && target_pos_y >= base_y_ - 1.0 && target_pos_y <= base_y_ + 1.0))
        {
            // down
            yaw_angle = M_PI;
            goal_x = target_pos_x + target_tolerance;
            goal_y = target_pos_y;
        }
        else if (target_pos_y >= max_y_ - 2.0 || (target_pos_y < base_y_ - 0.25 && target_pos_y > min_y_ + 2.0 && target_pos_x >= base_x_ - 0.25 && target_pos_x <= base_x_ + 0.25))
        {
            // left
            yaw_angle = M_PI / 2;
            goal_x = target_pos_x;
            goal_y = target_pos_y - target_tolerance;
        }
        else if (target_pos_y <= min_y_ + 2.0 || (target_pos_y > base_y_ + 0.25 && target_pos_y < max_y_ - 2.0 && target_pos_x >= base_x_ - 0.25 && target_pos_x <= base_x_ + 0.25))
        {
            // right
            yaw_angle = 3 * M_PI / 2;
            goal_x = target_pos_x;
            goal_y = target_pos_y + target_tolerance;
        }
        else
        {
            // left
            yaw_angle = M_PI / 2;
            goal_x = target_pos_x;
            goal_y = target_pos_y - target_tolerance;
        }

        tf::Quaternion goal_angle(0, 0, yaw_angle);

        geometry_msgs::Quaternion goal_orientation;
        tf::quaternionTFToMsg(goal_angle, goal_orientation);

        geometry_msgs::PoseStamped target_goal;
        target_goal.header.frame_id = "map";
        target_goal.pose.position.x = goal_x;
        target_goal.pose.position.y = goal_y;
        target_goal.pose.orientation = goal_orientation;

        destination_pub_.publish(target_goal);

        goal_published_ = true;

    }

    void PNPOnlyNode::ud_mount_state_callback(const control_msgs::JointControllerState::ConstPtr& msg)
    {
        ud_mount_state_msg_ = msg;
    }

    void PNPOnlyNode::l_hand_state_callback(const control_msgs::JointControllerState::ConstPtr& msg)
    {
        l_hand_state_msg_ = msg;
        double l_hand_process_value = l_hand_state_msg_->process_value;
        if (abs(l_hand_process_value - l_prev_value_) < 0.0001) l_hand_count_ += 1;
        if (l_hand_count_ > 30) l_hand_stopped_ = true;
        l_prev_value_ = l_hand_process_value;

    }

    void PNPOnlyNode::r_hand_state_callback(const control_msgs::JointControllerState::ConstPtr& msg)
    {
        r_hand_state_msg_ = msg;
    }

    void PNPOnlyNode::move_base_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
    {
        goal_result_msg_ = msg;
    }

    void PNPOnlyNode::current_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_pose_ = msg->pose.pose;
    }

    void PNPOnlyNode::gripper_lift(double ud_cmd)
    {
        std_msgs::Float64 ud_cmd_pub;
        ud_cmd_pub.data = ud_cmd;
        ud_mount_pub_.publish(ud_cmd_pub);

    }

    void PNPOnlyNode::gripper_grip(double L_cmd, double R_cmd)
    {
        std_msgs::Float64 L_cmd_pub;
        L_cmd_pub.data = L_cmd;
        l_hand_pub_.publish(L_cmd_pub);
        
        std_msgs::Float64 R_cmd_pub;
        R_cmd_pub.data = R_cmd;
        r_hand_pub_.publish(R_cmd_pub);

    }

    void PNPOnlyNode::go_backward()
    {
        geometry_msgs::Twist twist;
        twist.linear.x = -0.11;
        twist.angular.z = 0.04;
        twist_pub_.publish(twist);
    }

    void PNPOnlyNode::stop()
    {
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        twist_pub_.publish(twist);
    }

    void PNPOnlyNode::mainloop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
            
            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.frame_id = "map";
            initial_pose.pose.pose = current_pose_;
            initial_pose_pub_.publish(initial_pose);
            
            if (!finished_)
            {
                // Go to obj's destination
                if (go_dest_.start && !go_dest_.done)
                {
                    if (!goal_published_)
                    {
                        go_destination(object_map_pose_, object_tolerance_);
                        object_v_pos_ = object_map_pose_.pose.position.z;
                    }
                        

                    if (goal_result_msg_ != nullptr)
                    {
                        if (goal_result_msg_->status.status == 3)
                        {
                            
                            lifting_.start = true;
                                                        
                            goal_result_msg_ = nullptr;
                            goal_published_ = false;
                            go_dest_.start = false;
                            go_dest_.done = true;
                        }
                        else if (goal_result_msg_->status.status == 4)
                        {
                            goal_result_msg_ = nullptr;
                            goal_published_ = false;
                            go_dest_.start = false;
                            job_cancel_ = true;
                        }
                    }
                }


                // Move down the gripper
                if (lifting_.start && !lifting_.done)
                {
                    double target_height;

                    if (object_v_pos_ > 0.2) target_height = object_v_pos_ - 0.1;
                    else target_height = gripper_min_height_;
                    gripper_lift(target_height);
                    double ud_state_error = ud_mount_state_msg_->error;
                    double ud_set_point = ud_mount_state_msg_->set_point;
                    if (ud_set_point == target_height && abs(ud_state_error) < 0.018)
                    {
                        lifting_.start = false;
                        gripping_.start = true;
                        lifting_.done = true;
                        l_hand_stopped_ = false;
                        l_hand_count_ = 0;
                    }
                    
                }

                // Grab the obj
                if (gripping_.start && !gripping_.done)
                {
                    
                    const double offset = 0.05;

                    double r_cmd = (gripper_max_width_ - object_radius_[current_object_idx_] * 2) / 2 + offset;
                    double l_cmd = -r_cmd;
                
                    gripper_grip(l_cmd, r_cmd);

                    double l_hand_state_error = l_hand_state_msg_->error;
                    double l_hand_set_point = l_hand_state_msg_->set_point;
                    double r_hand_state_error = r_hand_state_msg_->error;
                    double r_hand_set_point = r_hand_state_msg_->set_point;
                    if ((l_hand_set_point == l_cmd && abs(l_hand_state_error) < 0.0165
                        && r_hand_set_point == r_cmd && abs(r_hand_state_error) < 0.0165)
                        || l_hand_stopped_)
                    {
                        gripping_.start = false;
                        lift_to_max_.start = true;
                        gripping_.done = true;
                        l_hand_stopped_ = false;
                        l_hand_count_ = 0;
                    }
                }

                // Lift up the gripper with the obj
                if (lift_to_max_.start && !lift_to_max_.done)
                {
                    gripper_lift(gripper_max_height_);
                    double ud_state_error = ud_mount_state_msg_->error;
                    double ud_set_point = ud_mount_state_msg_->set_point;
                    if (ud_set_point == gripper_max_height_ && abs(ud_state_error) < 0.1)
                    {
                        lift_to_max_.start = false;
                        go_base_.start = true;
                        lift_to_max_.done = true;
                    }
                }

                // Go to base box
                if (go_base_.start && !go_base_.done && base_counter_ < 2)
                {
                    geometry_msgs::PoseStamped base_pose;

                    if (base_counter_ == 0)
                    {
                        base_pose.header.frame_id = "map";
                        base_pose.pose.position.x = base_x_;
                        base_pose.pose.position.y = base_y_ - 2.0;
                        base_pose.pose.orientation.w = 1.0;
                    }
                    else if (base_counter_ == 1)
                    {
                        base_pose.header.frame_id = "map";
                        base_pose.pose.position.x = base_x_;
                        base_pose.pose.position.y = base_y_;
                        base_pose.pose.orientation.w = 1.0;
                    }
                    
                    if (!goal_published_)
                        go_destination(base_pose, base_tolerance_);

                    if (goal_result_msg_ != nullptr)
                    {
                        if (goal_result_msg_->status.status == 3)
                        {
                            base_counter_++;

                            goal_result_msg_ = nullptr;
                            goal_published_ = false;
                            if (base_counter_ == 2)
                            {
                                go_base_.start = false;
                                lift_to_place_.start = true;
                                go_base_.done = true;
                                base_counter_ = 0;
                                prev_pose_ = current_pose_;
                            }
                            
                        }
                        else if (goal_result_msg_->status.status == 4)
                        {
                            goal_result_msg_ = nullptr;
                            goal_published_ = false;
                            go_base_.start = false;
                            job_cancel_ = true;
                        }
                    }
                }

                // Lower the gripper to corresponding position
                if (lift_to_place_.start && !lift_to_place_.done)
                {
                    double target_height = object_height_ * (current_object_idx_ + 1) + 0.05;
                    gripper_lift(target_height);
                    double ud_state_error = ud_mount_state_msg_->error;
                    double ud_set_point = ud_mount_state_msg_->set_point;
                    if (ud_set_point == target_height && abs(ud_state_error) < 0.018)
                    {
                        lift_to_place_.start = false;
                        open_grip_.start = true;
                        lift_to_place_.done = true;
                    }
                }

                // Open the gripper
                if (open_grip_.start && !open_grip_.done)
                {
                    gripper_grip(0.0, 0.0);

                    double l_hand_state_error = l_hand_state_msg_->error;
                    double l_hand_set_point = l_hand_state_msg_->set_point;
                    if (l_hand_set_point == 0.0 && abs(l_hand_state_error) < 0.016)
                    {
                        open_grip_.start = false;
                        lift_to_max_2_.start = true;
                        open_grip_.done = true;
                    }
                }

                // Lift the gripper to the top
                if (lift_to_max_2_.start && !lift_to_max_2_.done)
                {
                    gripper_lift(gripper_max_height_);
                    double ud_state_error = ud_mount_state_msg_->error;
                    double ud_set_point = ud_mount_state_msg_->set_point;
                    if (ud_set_point == gripper_max_height_ && abs(ud_state_error) < 0.018)
                    {
                        lift_to_max_2_.start = false;
                        go_backward_.start = true;
                        lift_to_max_2_.done = true;
                    }
                }

                // Move backward and check places
                if (go_backward_.start && !go_backward_.done)
                {
                    // go backward
                    go_backward();
                    double distance_moved = sqrt(pow(current_pose_.position.x - prev_pose_.position.x, 2) + pow(current_pose_.position.y - prev_pose_.position.y, 2));

                    if (distance_moved > 1.0)
                    {
                        stop();
                        go_backward_.start = false;
                        go_backward_.done = true;
                        check_in_place_.start = true;
                    }
                }

                if (check_in_place_.start && !check_in_place_.done)
                {
                    // check the obj whether in places
                    
                    for (int i = 0; i < models_msg_->name.size(); i++)
                    {
                        std::string curr_model = models_msg_->name[i];
                        for (int j = 0; j < current_object_idx_ + 1; j++)
                        {
                            if (curr_model == object_left_[j])
                            {
                                geometry_msgs::PoseStamped object_pose;
                                object_pose.header.frame_id = tf_prefix_ + "/odom";
                                object_pose.pose = models_msg_->pose[i];

                                geometry_msgs::PoseStamped object_map_pose;

                                tf_odom_to_map(object_pose, object_map_pose);

                                if ((object_map_pose.pose.position.z > object_height_ * (j + 1) 
                                    && object_map_pose.pose.position.z < object_height_ * (j + 2))
                                
                                && ((object_map_pose.pose.position.x <= base_x_ + base_safety_offset_ && object_map_pose.pose.position.x >= base_x_ - base_safety_offset_)
                                    && (object_map_pose.pose.position.y <= base_y_ + base_safety_offset_ && object_map_pose.pose.position.y >= base_y_ - base_safety_offset_)))
                                {
                                    place_check_[j] = true;
                                }
                            }
                        }
                    }

                    if (!place_check_[current_object_idx_])
                    {
                        for (int m = 0; m < current_object_idx_ + 1; m++)
                        {
                            if (!place_check_[m])
                            {
                                current_object_idx_ = m;
                                job_cancel_ = true;
                                break;
                            }
                        }
                    }

                    for (int k = 0; k < current_object_idx_ + 1; k++) 
                            place_check_[k] = false;
                        
                    check_in_place_.start = false;
                    check_in_place_.done = true;
                }

                // Check all the jobs are done and reset
                int count_done = 0;
                job_list_ = 
                { 
                    go_dest_, 
                    lifting_, 
                    gripping_, 
                    lift_to_max_, 
                    go_base_, 
                    lift_to_place_, 
                    open_grip_,
                    lift_to_max_2_,
                    go_backward_,
                    check_in_place_
                };
                
                for (std::list<Job>::iterator job = job_list_.begin(); job != job_list_.end(); ++job)
                {
                    if (job->done == true) count_done += 1;
                }

                if (count_done == job_list_.size() || job_cancel_)
                { 
                    // reset job_done_
                    for (std::list<Job>::iterator job = job_list_.begin(); job != job_list_.end(); ++job)
                        job->done = false;

                    go_dest_.reset();
                    lifting_.reset();
                    gripping_.reset();
                    lift_to_max_.reset();
                    go_base_.reset();
                    lift_to_place_.reset();
                    open_grip_.reset();
                    lift_to_max_2_.reset();
                    go_backward_.reset();
                    check_in_place_.reset();
                        
                       
                    if (!job_cancel_ && current_object_idx_ < 5) 
                        current_object_idx_++;

                    if (current_object_idx_ == 5)
                    {
                        current_object_idx_ -= 1;
                        finished_ = true;
                    }
                    if (job_cancel_) gripper_grip(0.0, 0.0);

                    
                    job_cancel_ = false;
                }
            }
            
            
        }
        return;
    }
}