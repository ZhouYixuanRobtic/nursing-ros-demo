#ifndef NURSING_DRIVER_NURSING_DRIVER_H
#define NURSING_DRIVER_NURSING_DRIVER_H

#include "SocketCommunicator.h"
#include <thread>
#include <string>
#include <sys/timeb.h>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <industrial_msgs/RobotStatus.h>
#include "NursingMetaType.h"
#include "sensor_msgs/JointState.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#define BIG_MODULE_RATIO 2 * M_PI / 60.0 / 121
#define SMALL_MODULE_RATIO 2 * M_PI / 60.0 / 101
#define VMAX 3000
#define AMAX 10000
#define JMAX 40000
#define ARM_DOF 6
#define MAXALLOWEDDELAY 50
namespace nursing_driver
{
    class NursingDriver{
    private:
        const unsigned short SERVER_PORT=0x8888;
        const int TIMER_SPAN_RATE_ =50;
        const double THRESHOLD_ = 0.000001;

        void MoveItPoseCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);
        void trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg);
        void timerCallback(const ros::TimerEvent& e);
        bool setRobotJointsByMoveIt();

        double last_receive_point_[ARM_DOF]{};
        bool emergency_stopped_{};
        bool protective_stopped_{};
        bool normal_stopped_{};
        bool data_received_{};
        int data_count_{};
        bool real_robot_exist_{};
        bool controller_connected_flag_{};
        bool start_move_{};
        double current_joints_[ARM_DOF]{};
        double target_point_[ARM_DOF]{};

        SocketCommunicator::SocketClient *socket_client_;
        ros::NodeHandle nh_;
        ros::Publisher  cancle_trajectory_pub_;
        ros::Timer timer_;
        int collision_class_;
        industrial_msgs::RobotStatus robot_status_;

        ros::Publisher joint_states_pub_;
        ros::Publisher joint_feedback_pub_;
        ros::Publisher joint_target_pub_;
        ros::Publisher robot_status_pub_;
        ros::Subscriber moveAPI_subs_;
        ros::Subscriber moveIt_controller_subs_;
        ros::Subscriber trajectory_execution_subs_;
        ros::Subscriber robot_control_subs_;
        std::queue<nursing_namespace::PlanningState>  planning_buf_queue_{};
        nursing_namespace::PlanningState ps_{};
    public:
        const int UPDATE_RATE_ =400;
        NursingDriver();
        ~NursingDriver();
        bool roadPointCompare(const double *point1, const double *point2) const;
        double* getCurrentPosition();
        void setCurrentPosition(const double *target);
        double* getTargetPosition();
        void setTargetPosition(const double *target);
        void updateControlStatus();
        void run();
        bool connectToRobotController();
        static std::string joint_name_[ARM_DOF];
        double joint_ratio_[ARM_DOF]{};
        int buffer_size_;
    };
}



#endif //NURSING_DRIVER_NURSING_DRIVER_H
