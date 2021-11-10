#ifndef ROBOT_H
#define ROBOT_H

#include <signals.h>
#include <string>
#include <iostream>

#include <namespace_crcl_generated.h>
#include <types_crcl_generated_handling.h>

#include "DRFLEx.h"
#include <Eigen/Dense>

class Robot
{
public:
    Robot(std::string address_, Signals* signals_);
    ~Robot()=default;
    DRAFramework::CDRFLEx drfl;

    int ExecuteMoveTo(UA_MoveToParametersSetDataType *moveTo);
    int ConnectToDoosan(std::string ip);
private:
    Signals* signals;
    std::string address;
};

#endif // ROBOT_H
