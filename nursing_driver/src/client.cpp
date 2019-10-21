#include <sensor_msgs/JointState.h>
#include "SocketCommunicator.h"
#include "ros/ros.h"
int main(int argc, char * argv[])
{
    ros::init(argc,argv,"client");
    ros::NodeHandle nh;
    const unsigned short port_num=0x8888;
    SocketCommunicator::SocketClient sc(port_num);
    char *clientContent= new char[150];
    nursing_namespace::PlanningState ps{};
    sc.start();
    bool & emergency_bool = sc.receivedBool();
    sc.beginListen();
    int i=0;
    sensor_msgs::JointState jointState{};
    ros::Rate loop_rate(400);
    while(sc.isConnectionOk_)
    {
        ps=sc.getPlanningState();

        jointState.header.stamp=ros::Time::now();
        static int write_counter=0;
        memcpy(clientContent,&ps,sizeof(nursing_namespace::PlanningState));
        sc.write(clientContent, sizeof(nursing_namespace::PlanningState));
        write_counter++;
        ROS_INFO_STREAM(write_counter);
        loop_rate.sleep();

    }
    delete [] clientContent;
    return 0;
}