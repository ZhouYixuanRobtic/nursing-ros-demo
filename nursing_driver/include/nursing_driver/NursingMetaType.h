//
// Created by xcy on 2019/9/26.
//

#ifndef NURSING_DRIVER_NURSINGMETATYPE_H
#define NURSING_DRIVER_NURSINGMETATYPE_H
namespace nursing_namespace{
    enum {
        ARM_DOF=6,
    };

    struct PlanningState{
        double joint_pos_[ARM_DOF];
        double joint_vel_[ARM_DOF];
        double joint_acc_[ARM_DOF];
    };
}
#endif //NURSING_DRIVER_NURSINGMETATYPE_H
