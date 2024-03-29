#include "SocketCommunicator.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
nursing_namespace::PlanningState ps{};
#define  ARM_DOF 6
void joint_states_CB(const sensor_msgs::JointState &msg)
{
    boost::mutex callback_mutex;
    auto joints= new double[ARM_DOF];
    joints[0]=msg.position[3];
    joints[1]=msg.position[0];
    joints[2]=msg.position[1];
    joints[3]=msg.position[4];
    joints[4]=msg.position[5];
    joints[5]=msg.position[2];
    callback_mutex.lock();
    memcpy(ps.joint_pos_,joints,sizeof(ps.joint_pos_));
    memcpy(ps.joint_vel_,msg.velocity.data(),sizeof(ps.joint_vel_));
    memcpy(ps.joint_acc_,msg.effort.data(),sizeof(ps.joint_acc_));
    callback_mutex.unlock();
    delete [] joints;
}
int main(int argc, char * argv[])
{
    ros::init(argc,argv,"server");
    ros::NodeHandle nh;
    ros::Rate loop_rate(400);

    const unsigned short port_num=0x8888;
    SocketCommunicator::SocketServer sc(port_num);
    char serverContent[150];
    std::string robot_name;
    ros::param::get("/robot_name",robot_name);

    ros::Subscriber joint_states_sub=nh.subscribe("/"+robot_name+"/joint_states",100,joint_states_CB);
    ros::Publisher received_command_pub = nh.advertise<sensor_msgs::JointState>("/receive_command",100);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sc.registerServerReadThread(50);
    sc.registerServerWriteThread(&ps,50);

    nursing_namespace::PlanningState temp_ps{};
    sensor_msgs::JointState jointState{};
    bool a=true;
    while(ros::ok())
    {
        if(!sc.isInitialized)
        {
            sc.initialize();
            continue;
        }
        if(!sc.isConnectionOk)
            sc.waitForConnection();

        temp_ps=sc.getPlanningState();
        jointState.header.stamp=ros::Time::now();
        jointState.position.resize(ARM_DOF);
        jointState.header.seq++;
        for(int i=0;i<ARM_DOF;++i)
            jointState.position[i]=temp_ps.joint_pos_[i];
        received_command_pub.publish(jointState);
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
