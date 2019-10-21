#include "NursingDriver.h"
namespace nursing_driver
{
NursingDriver::NursingDriver() : buffer_size_(400),collision_class_(6),jti_(ARM_DOF,1.0/200),jto_(ARM_DOF)
{
    socket_client_ = new SocketCommunicator::SocketClient(SERVER_PORT);
    socket_client_->start();
    for(int i=0; i < ARM_DOF;++i)
    {
        //To Do: The value of MODULE_RATIO needed adjust by the real application.
        if(i < 3)
            joint_ratio_[i]=BIG_MODULE_RATIO;
        else
            joint_ratio_[i]=SMALL_MODULE_RATIO;
        jti_.maxVelocity[i] = VMAX *joint_ratio_[i];
        jti_.maxAcceleration[i] = AMAX*joint_ratio_[i];
        jti_.maxJerk[i]= JMAX*joint_ratio_[i];
    }
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);
    joint_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/nursing_driver/real_pose", 50);
    cancel_trajectory_pub_ = nh_.advertise<std_msgs::UInt8>("/nursing_driver/cancel_trajectory", 100);

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
            auto joints = new double[ARM_DOF];
            memcpy(joints,ps_.joint_pos_,ARM_DOF* sizeof(double));
            setCurrentPosition(joints);
            delete[] joints;
        }
        else
            ROS_ERROR("Can't connect to the robot controller!");
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
                cancel_trajectory_pub_.publish(cancel);
                memcpy(&jti_.currentPosition[0], temp_ps.joint_pos_, ARM_DOF*sizeof(double));
                memcpy(&jti_.currentVelocity[0], temp_ps.joint_vel_, ARM_DOF*sizeof(double));
                memcpy(&jti_.currentAcceleration[0], temp_ps.joint_acc_, ARM_DOF*sizeof(double));
                memset(&jti_.targetVelocity[0],0,ARM_DOF* sizeof(double));
                bool update = otgVelocityModeParameterUpdate(jti_);
                int resultValue = 0;
                while(resultValue != 1)
                {
                    resultValue = otgVelocityModeResult(1,jto_);
                    auto clientContent = new char[150];
                    memcpy(clientContent,jto_.newPosition.data(),sizeof(double)*ARM_DOF);
                    socket_client_->write(clientContent, sizeof(double)*ARM_DOF);
                    delete[] clientContent;
                }
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
                auto clientContent = new char[150];
                memcpy(clientContent,temp_ps.joint_pos_,sizeof(double)*ARM_DOF);
                socket_client_->write(clientContent, sizeof(double)*ARM_DOF);
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
    memcpy(jointAngle,msg->positions.data(),sizeof(double)*ARM_DOF);
    if(controller_connected_flag_)
    {
        if(roadPointCompare(jointAngle,last_receive_point_))
        {
            ROS_DEBUG("Add new waypoint to the buffer.");
            data_count_=0;
            nursing_namespace::PlanningState ps{};
            memcpy(ps.joint_pos_,jointAngle, sizeof(double)*ARM_DOF);
            memcpy(ps.joint_vel_,msg->velocities.data(),sizeof(double)*ARM_DOF);
            memcpy(ps.joint_acc_,msg->accelerations.data(),sizeof(double)*ARM_DOF);
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
        ros::param::set("/nursing_driver/robot_connected","1");
    }
    else
    {
        ROS_INFO("Didn't connect to robot, did you launch the real robot?");
        ros::param::set("/nursing_driver/robot_connected","0");
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_RATE_),&NursingDriver::timerCallback,this);
    timer_.start();
}

}
