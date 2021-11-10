#include "robot.h"


Robot::Robot(std::string address_, Signals* signals_):
    address(address_),
    signals(signals_)
    {
    }

int Robot::ExecuteMoveTo(UA_MoveToParametersSetDataType* moveTo){
    std::cout << "\nRobot:\nExecute MoveTo command\n" << std::endl;
    // Convert the rotation Matrix to euler angle
    Eigen::Vector3d xAxis(moveTo->endPosition.xAxis.i,
                          moveTo->endPosition.xAxis.j,
                          moveTo->endPosition.xAxis.k);
    Eigen::Vector3d zAxis(moveTo->endPosition.zAxis.i,
                          moveTo->endPosition.zAxis.j,
                          moveTo->endPosition.zAxis.k);
    Eigen::Vector3d yAxis = xAxis.cross(zAxis);
    Eigen::Matrix<double, 3, 3> rotMatrix;
    rotMatrix << xAxis(0), xAxis(1), xAxis(2),
                 yAxis(0), yAxis(1), yAxis(2),
                 zAxis(0), zAxis(1), zAxis(2);
    Eigen::Vector3d euler = rotMatrix.eulerAngles(0,1,2);

    if (moveTo->moveStraight){
        float x1[6] = { moveTo->endPosition.point.x, moveTo->endPosition.point.y,
                        moveTo->endPosition.point.z, euler(0), euler(1), euler(2)};
        float tvel = 50; // to get the vellocity from the samy core use SetTransSpeed before a move command
        float tacc = 100; // To get the acceleration from the samy core use SetTransAccel before a move command
        drfl.movel(x1, &tvel, &tacc);
    } else {
        float x1[6] = { moveTo->endPosition.point.x, moveTo->endPosition.point.y,
                        moveTo->endPosition.point.z, euler(0), euler(1), euler(2)};
        float sol=2;
        float jvel=10;
        float jacc=20;
        drfl.movejx(x1, sol, jvel, jacc);
    }
    return 1;
}


int Robot::ConnectToDoosan(std::string ip){
    std::cout << "Try to connect to Robot" << std::endl;
    drfl.open_connection(ip);
    std::cout << "Connection established" << std::endl;
    drfl.set_robot_mode(ROBOT_MODE_MANUAL);
    drfl.change_operation_speed(100);
    return 1;
}
