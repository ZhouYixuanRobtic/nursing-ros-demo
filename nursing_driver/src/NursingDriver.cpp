#include "NursingDriver.h"
namespace nursing_driver
{
std::string NursingDriver::joint_name_[ARM_DOF]={"shoulder_joint", "bigarm_joint", "elbow_joint",
                                                 "wrist1_joint", "wrist2_joint", "palm_joint"};

NursingDriver::NursingDriver() : buffer_size_(400),collision_class_(6)
{
    socket_client_ = new SocketCommunicator::SocketClient(SERVER_PORT);
    socket_client_->start();
    for(int i=0; i < ARM_DOF;++i)
    {
        if(i < 3)
            joint_ratio_[i]=BIG_MODULE_RATIO;
        else if(i < 6)
            joint_ratio_[i]=SMALL_MODULE_RATIO;
        else
            joint_ratio_[i]=2 * M_PI / 10.05309632;
    }
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);
    joint_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/nursing_driver/real_pose", 50);
    cancle_trajectory_pub_ = nh_.advertise<std_msgs::UInt8>("/nursing_driver/cancel_trajectory",100);

    /** subscribe topics **/
    trajectory_execution_subs_ = nh_.subscribe("trajectory_execution_event", 10, &NursingDriver::trajectoryExecutionCallback,this);
    moveIt_controller_subs_ = nh_.subscribe("moveItController_cmd", 2000, &NursingDriver::MoveItPoseCallback,this);
}
NursingDriver::~NursingDriver()
{
    delete socket_client_;
}
void NursingDriver::timerCallback(const ros::TimerEvent &e)
{
    if(controller_connected_flag_)
    {
        if(socket_client_->isConnectionOk_)
        {
            ps_=socket_client_->getPlanningState();
            static bool firstTime=true;
            if(firstTime)
            {
                std::cout<<"DRIVER TIMER"<<ps_.joint_pos_[1]<<std::endl;
                firstTime=false;
            }

            auto joints = new double[ARM_DOF];
            memcpy(joints,ps_.joint_pos_,ARM_DOF* sizeof(double));
            setCurrentPosition(joints);
            delete[] joints;
        }
        else
        {
            ROS_ERROR("Can't connect to the robot controller!");
        }
    }
    else
        setCurrentPosition(target_point_);

    sensor_msgs::JointState joint_state;
    control_msgs::FollowJointTrajectoryFeedback joint_feedback;

    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(ARM_DOF);
    joint_feedback.joint_names.resize(ARM_DOF);
    joint_state.position.resize(ARM_DOF);
    joint_feedback.actual.positions.resize(ARM_DOF);
    for(int i = 0; i<ARM_DOF; i++)
    {
        joint_state.name[i] = joint_name_[i];
        if (controller_connected_flag_)
            joint_state.position[i] = current_joints_[i];
        else
            joint_state.position[i] = target_point_[i];

        joint_feedback.joint_names[i] = joint_name_[i];
        joint_feedback.actual.positions[i] = joint_state.position[i];
    }
    joint_states_pub_.publish(joint_state);
    joint_feedback_pub_.publish(joint_feedback);
}
bool NursingDriver::roadPointCompare(const double *point1, const double *point2) const
{
    for(int i = 0; i < ARM_DOF;i++)
    {
        if(fabs(point1[i] - point2[i]) >= THRESHOLD_)
            return true;
    }
    return false;
}
double* NursingDriver::getCurrentPosition()
{
    return current_joints_;
}
void NursingDriver::setCurrentPosition(const double *target)
{
    for(int i = 0; i < ARM_DOF;i++)
    {
        current_joints_[i] = target[i];
    }
}
double* NursingDriver::getTargetPosition()
{
    return target_point_;
}
void NursingDriver::setTargetPosition(const double *target)
{
    for(int i = 0; i < ARM_DOF;i++)
    {
        target_point_[i] = target[i];
    }
}
bool NursingDriver::setRobotJointsByMoveIt()
{
    if(!planning_buf_queue_.empty())
    {
        nursing_namespace::PlanningState temp_ps= planning_buf_queue_.front();
        planning_buf_queue_.pop();
        if(controller_connected_flag_)
        {
            if(emergency_stopped_)
            {
                start_move_=false;
                while(!planning_buf_queue_.empty())
                    planning_buf_queue_.pop();
            }
            else if(protective_stopped_||normal_stopped_)
            {
                std_msgs::UInt8 cancel;
                cancel.data = 1;
                cancle_trajectory_pub_.publish(cancel);
                /*
                 * to do:
                 *      first slow down until the velocity to zero, use a while loop
                 */
                auto clientContent = new char[150];
                memcpy(clientContent,&temp_ps,sizeof(nursing_namespace::PlanningState));
                socket_client_->write(clientContent, sizeof(nursing_namespace::PlanningState));
                delete[] clientContent;
                start_move_ = false;
                //clear buffer
                while(!planning_buf_queue_.empty())
                    planning_buf_queue_.pop();
                //clear flag
                if(normal_stopped_)
                    normal_stopped_=false;
            }
            else
            {
                ROS_INFO_ONCE("WRITE DOWN WITH THE FIRST JOINT: %lf",temp_ps.joint_pos_[0]);
                auto clientContent = new char[150];
                memcpy(clientContent,&temp_ps,sizeof(nursing_namespace::PlanningState));
                socket_client_->write(clientContent, sizeof(nursing_namespace::PlanningState));
                delete[] clientContent;
            }
        }
        setTargetPosition(temp_ps.joint_pos_);
    }
    else
    {
        if(start_move_)
            start_move_=false;
    }
}
void NursingDriver::MoveItPoseCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    auto jointAngle=new double[ARM_DOF];
    memcpy(jointAngle,&msg->positions[0],sizeof(double)*ARM_DOF);
    if(controller_connected_flag_)
    {
        ROS_INFO_ONCE("out the compare");
        if(roadPointCompare(jointAngle,last_receive_point_))
        {
            ROS_INFO_ONCE("inside the compare");
            ROS_DEBUG("Add new waypoint to the buffer.");
            data_count_=0;
            nursing_namespace::PlanningState ps{};
            memcpy(ps.joint_pos_,jointAngle, sizeof(double)*ARM_DOF);
            memcpy(ps.joint_vel_,&msg->velocities[0],sizeof(double)*ARM_DOF);
            memcpy(ps.joint_acc_,&msg->accelerations[0],sizeof(double)*ARM_DOF);
            planning_buf_queue_.push(ps);
            if(planning_buf_queue_.size()>buffer_size_&&!start_move_)
                start_move_=true;
        }
    }
    else
    {
        setTargetPosition(jointAngle);
    }
    delete [] jointAngle;
}
void NursingDriver::trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "stop")
    {
        ROS_INFO("trajectory execution status: stop");
        normal_stopped_ = true;
    }
}
void NursingDriver::updateControlStatus()
{
    data_count_++;
    if(data_count_ == MAXALLOWEDDELAY)
    {
        data_count_=0;
        if(!planning_buf_queue_.empty()&&!start_move_)
            start_move_=true;
    }
    if(start_move_)
        setRobotJointsByMoveIt();
}
bool NursingDriver::connectToRobotController()
{
    controller_connected_flag_=socket_client_->beginListen();
    return controller_connected_flag_;
}
void NursingDriver::run()
{
    ROS_INFO("Start the driver");
    if(connectToRobotController())
    {
        ROS_INFO("Connected!!");
        nursing_namespace::PlanningState temp_ps=socket_client_->getPlanningState();
        auto joints = new double[ARM_DOF];
        memcpy(joints,temp_ps.joint_pos_,ARM_DOF*sizeof(double));
        setCurrentPosition(joints);
        setTargetPosition(joints);
        std_msgs::Float32MultiArray robot_joints;
        robot_joints.data.resize(ARM_DOF);
        for(int i=0; i<ARM_DOF;++i)
            robot_joints.data[i]=current_joints_[i];
        joint_target_pub_.publish(robot_joints);
        ros::param::set("initial_joint_state",joints);
        delete[] joints;
    }
    ros::param::set("/nursing_driver/robot_connected","1");
    timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_RATE_),&NursingDriver::timerCallback,this);
    timer_.start();
}

}
