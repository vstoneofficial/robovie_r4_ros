#include <r4_control/r4_hw_interface.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <std_msgs/Float32MultiArray.h>

namespace r4_control_ns
{
    R4HWInterface::R4HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        sensor_sub = nh.subscribe("/r4/r4Sensor", 1, &R4HWInterface::sensorCallback, this);
        goal_pos_sub_l_arm = nh.subscribe("/l_arm_position_trajectory_controller/follow_joint_trajectory/goal", 1, &R4HWInterface::lArmGoalCallback, this);
        goal_pos_sub_r_arm = nh.subscribe("/r_arm_position_trajectory_controller/follow_joint_trajectory/goal", 1, &R4HWInterface::rArmGoalCallback, this);
        goal_pos_sub_head = nh.subscribe("/head_position_trajectory_controller/follow_joint_trajectory/goal", 1, &R4HWInterface::headGoalCallback, this);

        cmd_pub = nh.advertise<r4_control::r4Cmd>("/r4/r4Cmd", 3);
        ROS_INFO("R4HWInterface declared.");
    }

    void R4HWInterface::lArmGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        int waypoints = msg->goal.trajectory.points.size();

        final_angle[0] = msg->goal.trajectory.points[waypoints - 1].positions[2];  // shoulder_pitch
        final_angle[1] = msg->goal.trajectory.points[waypoints - 1].positions[3];  // shoulder_roll
        final_angle[2] = msg->goal.trajectory.points[waypoints - 1].positions[1];  // elbow_yaw
        final_angle[3] = msg->goal.trajectory.points[waypoints - 1].positions[0];  // elbow_bend

        duration = msg->goal.trajectory.points[waypoints - 1].time_from_start.toSec();
    }

    void R4HWInterface::rArmGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        int waypoints = msg->goal.trajectory.points.size();

        final_angle[4] = msg->goal.trajectory.points[waypoints - 1].positions[2];  // shoulder_pitch
        final_angle[5] = msg->goal.trajectory.points[waypoints - 1].positions[3];  // shoulder_roll
        final_angle[6] = msg->goal.trajectory.points[waypoints - 1].positions[1];  // elbow_yaw
        final_angle[7] = msg->goal.trajectory.points[waypoints - 1].positions[0];  // elbow_bend

        duration = msg->goal.trajectory.points[waypoints - 1].time_from_start.toSec();
    }

    // want: yaw roll pitch
    // got: pitch roll yaw
    void R4HWInterface::headGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr &msg)
    {
        int waypoints = msg->goal.trajectory.points.size();

        final_angle[8] = msg->goal.trajectory.points[waypoints - 1].positions[2];  //neck_yaw
        final_angle[9] = msg->goal.trajectory.points[waypoints - 1].positions[1];  //neck_roll
        final_angle[10] = msg->goal.trajectory.points[waypoints - 1].positions[0];  //neck_pitch

        duration = msg->goal.trajectory.points[waypoints - 1].time_from_start.toSec();
    }

    void R4HWInterface::sensorCallback(const r4_control::r4Sensor::ConstPtr &msg)
    {
        for (int i = 0; i < num_joints_; i++) {
            joint_position_[i] = msg->angle[i] * DEG_TO_RAD / 10.0; // angle received is multiplied by 10
        }
    }

    void R4HWInterface::init()
    {
        // Call parent class version of this function
        ros_control_boilerplate::GenericHWInterface::init();

        final_angle.resize(joint_position_.size());
        joint_position_prev_.resize(joint_position_.size());
        joint_position_command_.resize(joint_position_.size());

        ROS_INFO("R4HWInterface initiated.");
    }

    void R4HWInterface::read(ros::Duration &elapsed_time)
    {
        // No need to read since our write() command populates our state for us
        // ros::spinOnce();
    }

    void R4HWInterface::write(ros::Duration &elapsed_time)
    {
        static r4_control::r4Cmd joint_cmd;

        bool change_detected = false;
        for (int i = 0; i < num_joints_; i++) {
            if (joint_position_prev_[i] != joint_position_command_[i]) {
                change_detected = true;
                i = (int)num_joints_;
            }
        }

        if (change_detected) {
            for (int i = 0; i < num_joints_; i++) {
                joint_cmd.angle[i] = (int16_t)(final_angle[i] * RAD_TO_DEG * 10);
                joint_position_prev_[i] = joint_position_command_[i];
            }
            joint_cmd.time = (int16_t)(duration * 1000 / 16.667);
            cmd_pub.publish(joint_cmd);
        }
    }

    void R4HWInterface::enforceLimits(ros::Duration &period)
    {
        // Enforces position and velocity
        // pos_jnt_sat_interface_.enforceLimits(period);
    }

} // namespace r4_control_ns
