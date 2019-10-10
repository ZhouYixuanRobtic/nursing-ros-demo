#include "SocketCommunicator.h"
#include "ros/ros.h"
int main(int argc, char * argv[])
{

    const unsigned short port_num=0x8888;
    SocketCommunicator::SocketClient sc(port_num);
    char *clientContent= new char[150];
    nursing_namespace::PlanningState ps{};
    sc.start();
    sc.beginListen();
    int i=0;
    while(sc.isConnectionOk_)
    {
        ps=sc.getPlanningState();
        std::cout<<i<<","<<ps.joint_pos_[0]<<std::endl;
        usleep(1E6/60);
        if(i<INT_MAX)
            i++;
        else
            i=0;
    }
    delete [] clientContent;
    return 0;
}