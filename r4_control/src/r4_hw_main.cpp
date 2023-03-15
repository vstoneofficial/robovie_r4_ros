#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <r4_control/r4_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "r4_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::shared_ptr<r4_control_ns::R4HWInterface> r4_hw_interface_instance(
      new r4_control_ns::R4HWInterface(nh));
  r4_hw_interface_instance->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, r4_hw_interface_instance);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}