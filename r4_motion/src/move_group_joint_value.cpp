/*
 * 各軸の目標角度を指定してRobovie-R4を動かすサンプル
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// 最大関節速度、加速度を下げるためのスケーリング係数の設定。許可される値は0～1です。
// ロボットモデルで指定された最大関節速度に係数が乗算されます。
const double MAX_VEL_ARM = 0.4;
const double MAX_ACC_ARM = 0.5;
const double MAX_VEL_HEAD = 1.0;
const double MAX_ACC_HEAD = 1.0;

void moveArm(moveit::planning_interface::MoveGroupInterface& group, std::vector<double>& angles, double delay, double maxVelScalingFactor)
{
    std::vector<double> arm_pose = {tf2Radians(angles[0]), 
                                    tf2Radians(angles[1]),
                                    tf2Radians(angles[2]),
                                    tf2Radians(angles[3])};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(arm_pose);
    ROS_INFO("Moving single arm");
    group.move();
    ros::Duration(delay).sleep();
}

void moveBothArms(moveit::planning_interface::MoveGroupInterface& group, std::vector<double>& angles, double delay, double maxVelScalingFactor)
{
    std::vector<double> arm_pose = {tf2Radians(angles[0]), 
                                    tf2Radians(angles[1]),
                                    tf2Radians(angles[2]),
                                    tf2Radians(angles[3]),
                                    tf2Radians(angles[4]),
                                    tf2Radians(angles[5]),
                                    tf2Radians(angles[6]),
                                    tf2Radians(angles[7])};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(arm_pose);
    ROS_INFO("Moving both arms");
    group.move();
    ros::Duration(delay).sleep();
}

void moveAllJoints(moveit::planning_interface::MoveGroupInterface& group, std::vector<double>& angles, double delay, double maxVelScalingFactor)
{
    std::vector<double> body_pose = {tf2Radians(angles[0]), 
                                    tf2Radians(angles[1]),
                                    tf2Radians(angles[2]),
                                    tf2Radians(angles[3]),
                                    tf2Radians(angles[4]),
                                    tf2Radians(angles[5]),
                                    tf2Radians(angles[6]),
                                    tf2Radians(angles[7]),
                                    tf2Radians(angles[8]),
                                    tf2Radians(angles[9]),
                                    tf2Radians(angles[10])};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(body_pose);
    group.setPlanningTime(10);
    ROS_INFO("Moving all joints");
    group.move();
    ros::Duration(delay).sleep();
}

void moveGripper(moveit::planning_interface::MoveGroupInterface& group, double angle, double delay, double maxVelScalingFactor)
{
    std::vector<double> gripper_pose = {tf2Radians(angle), 
                                    tf2Radians(angle),
                                    tf2Radians(angle)};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(gripper_pose);
    ROS_INFO("Moving head");
    group.move();
    ros::Duration(delay).sleep();
}

void inverted(std::vector<double>& angles, std::vector<double>& out_angles)
{
    for (auto &i : angles) out_angles.push_back(-i); 
}

void combine(std::vector<double>& l_angles, std::vector<double>& r_angles, std::vector<double>& out_angles)
{
    out_angles.insert(out_angles.begin(), r_angles.begin(), r_angles.end());
    out_angles.insert(out_angles.begin(), l_angles.begin(), l_angles.end());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_group_joint_value_sample");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit_visual_tools::MoveItVisualTools visual_tools("torso_link_r4");

    // creating planning group
    moveit::planning_interface::MoveGroupInterface larm("left_arm");
    moveit::planning_interface::MoveGroupInterface rarm("right_arm");
    moveit::planning_interface::MoveGroupInterface barm("both_arms");
    moveit::planning_interface::MoveGroupInterface head("head");
    moveit::planning_interface::MoveGroupInterface all("whole_body");

    larm.setMaxVelocityScalingFactor(MAX_VEL_ARM);
    larm.setMaxAccelerationScalingFactor(MAX_ACC_ARM);

    // get current joint angles
    std::vector<double> l_arm_out = {-90, 29, -69, -34};
    std::vector<double> home = {0, 0, 0, 0};
    std::vector<double> b_home = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> l_self = {-68, 12, -20, -77};
    std::vector<double> r_self;
    std::vector<double> r_arm_out;
    std::vector<double> b_arm_out;
    inverted(l_arm_out, r_arm_out);
    inverted(l_self, r_self);
    combine(l_arm_out, r_arm_out, b_arm_out);

    /***  pose for all joints ***************/
    std::vector<double> a_pose_0 = {0, 0, 0, 0,
                                    0, 0, 0,
                                    0, 0, 0, 0};
    std::vector<double> a_pose_1 = {-90, 29, -69, -34,
                                    -47, 7, -25,
                                    90, -29, 69, 34};
    std::vector<double> a_pose_2 = {-62, 44, -13, 97,
                                    35, -10, 5,
                                    63, -86, 46, 74};

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    int ctr = 0;

    while (ros::ok()) {
        printf("loop: %d\n", ctr);

        moveAllJoints(all, a_pose_0, 1.20, 0.5);
        moveAllJoints(all, a_pose_1, 1.20, 0.5);
        moveAllJoints(all, a_pose_2, 1.20, 0.5);

        ctr++;
    }
}