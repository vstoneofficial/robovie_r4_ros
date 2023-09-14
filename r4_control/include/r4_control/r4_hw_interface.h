#ifndef R4_HW_INTERFACE_H
#define R4_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <r4_control/r4Cmd.h>
#include <r4_control/r4Sensor.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>


const double DEG_TO_RAD = 0.017453292519943295769236907684886;
const double RAD_TO_DEG = 57.295779513082320876798154814105;

namespace r4_control_ns
{
/** \brief Hardware interface for a robot */
class R4HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  R4HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  ros::Subscriber sensor_sub;
  void sensorCallback(const r4_control::r4Sensor::ConstPtr& msg);
  void lArmGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& goal);
  void rArmGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& goal);
  void headGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& goal);


  ros::Publisher cmd_pub;
  std::vector<double> joint_position_prev_;
  std::vector<double> final_angle;
  double duration;
  // int interp_time = 0;
//   const float maxAngle[6] = {170.0,     0.0, 160.0, 120.0,  158.0, 60.0};
//   const float minAngle[6] = {-170.0, -135.0,   0.0, -75.0, -158.0, -15.0};


};  // class

}  // namespace ros_control_boilerplate

#endif
