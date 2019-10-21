#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define  ARM_DOF 6

float *joint_states;

void get_joint_states(const sensor_msgs::JointStateConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE_NAMED("GAZEBO DRIVER","Already get into callback");
    for(int i=0;i<ARM_DOF;i++)
    {
        //ROS_ERROR_STREAM_NAMED("gazebo"," : "<<msg->name[i] <<" : " << msg->position[i]);
        joint_states[i] = msg->position[i];
    }
    ROS_INFO_STREAM_ONCE_NAMED("GAZEBO DRIVER DATA",joint_states[1]);
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"nursing_gazebo_driver");
    ros::NodeHandle nh;
    joint_states = new float[ARM_DOF];

    ros::Subscriber joint_state_sub = nh.subscribe("/receive_command",10, &get_joint_states);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string robot_name;
    ros::param::get("/robot_name",robot_name);

    std::string shoulder_joint_command_name="/shoulder_joint_position_controller/command";
    std::string bigarm_joint_command_name="/bigarm_joint_position_controller/command";
    std::string elbow_joint_command_name="/elbow_joint_position_controller/command";
    std::string wrist1_joint_command_name="/wrist1_joint_position_controller/command";
    std::string wrist2_joint_command_name="/wrist2_joint_position_controller/command";
    std::string palm_joint_command_name="/palm_joint_position_controller/command";

    std::string shoulder_command_topic = robot_name+shoulder_joint_command_name;
    std::string bigarm_command_topic = robot_name+bigarm_joint_command_name;
    std::string elbow_command_topic  = robot_name+elbow_joint_command_name;
    std::string wrist1_command_topic   = robot_name+wrist1_joint_command_name;
    std::string wrist2_command_topic   = robot_name+wrist2_joint_command_name;
    std::string palm_command_topic   = robot_name+palm_joint_command_name;

    ROS_INFO_STREAM_NAMED("test_Topic_name",shoulder_command_topic);

    ros::Publisher  pub_gazebo_shoulder_joint = nh.advertise<std_msgs::Float64>(shoulder_command_topic,1000);
    ros::Publisher  pub_gazebo_bigarm_joint = nh.advertise<std_msgs::Float64>(bigarm_command_topic,1000);
    ros::Publisher  pub_gazebo_elbow_joint =  nh.advertise<std_msgs::Float64>(elbow_command_topic,1000);
    ros::Publisher  pub_gazebo_wrist1_joint =   nh.advertise<std_msgs::Float64>(wrist1_command_topic,1000);
    ros::Publisher  pub_gazebo_wrist2_joint =   nh.advertise<std_msgs::Float64>(wrist2_command_topic,1000);
    ros::Publisher  pub_gazebo_palm_joint =   nh.advertise<std_msgs::Float64>(palm_command_topic,1000);

    std_msgs::Float64 shoulder_joint{};
    std_msgs::Float64 bigarm_joint{};
    std_msgs::Float64 elbow_joint{};
    std_msgs::Float64 wrist1_joint{};
    std_msgs::Float64 wrist2_joint{};
    std_msgs::Float64 palm_joint{};

    while(ros::ok())
    {
        shoulder_joint.data = joint_states[0];
        bigarm_joint.data = joint_states[1];
        elbow_joint.data = joint_states[2];
        wrist1_joint.data = joint_states[3];
        wrist2_joint.data = joint_states[4];
        palm_joint.data = joint_states[5];
        pub_gazebo_shoulder_joint.publish(shoulder_joint);
        pub_gazebo_bigarm_joint.publish(bigarm_joint);
        pub_gazebo_elbow_joint.publish(elbow_joint);
        pub_gazebo_wrist1_joint.publish(wrist1_joint);
        pub_gazebo_wrist2_joint.publish(wrist2_joint);
        pub_gazebo_palm_joint.publish(palm_joint);

        ros::spinOnce();
        usleep(1000*100);
    }
    spinner.stop();
    ros::shutdown();
    delete[] joint_states;
}