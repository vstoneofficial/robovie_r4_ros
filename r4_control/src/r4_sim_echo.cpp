/*
* /r4/r4Cmdのデータを/r4/jointSensorに入れ、
* 疑似的にROS controlでr4をRviz上で動かすためのノード
* 
*/

#include <ros/ros.h>
#include <r4_control/r4Cmd.h>
#include <r4_control/r4Sensor.h>

ros::Publisher sensor_pub;

void cmdCallback(const r4_control::r4Cmd::ConstPtr& msg)
{
    static r4_control::r4Sensor sensor;

    for(int i = 0; i < msg->angle.size(); i++)
    {
        sensor.angle[i] = msg->angle[i];
        // sensor.vel[i] = msg->vel[i] / 1000;
    }
    sensor_pub.publish(sensor);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "r4_sim_echo");

    ros::NodeHandle n;

    ros::Subscriber cmd_sub = n.subscribe("/r4/r4Cmd", 10, cmdCallback);
    sensor_pub = n.advertise<r4_control::r4Sensor>("/r4/r4Sensor", 10);

    ros::spin();
}

