#include "NursingDriver.h"
#include <string>
#include <sys/timeb.h>
#include "NursingDriver.cpp"
using namespace nursing_driver;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nursing_driver");
    ros::NodeHandle n;

    NursingDriver robot_driver;
    robot_driver.run();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop_rate(robot_driver.UPDATE_RATE);
    while(ros::ok())
    {
        robot_driver.updateControlStatus();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_WARN("Exiting robot_driver");
    return(0);
}
