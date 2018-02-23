#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <motoman_mh12/TestHelpers.hpp>
#include <iostream>
#include <math.h>

using namespace motoman_mh12;

int main(int argc, char **argv)
{
    Driver driver_ctrl;
    driver_ctrl.openTCP("192.168.10.77", 50240);

    Driver driver_streaming;
    driver_streaming.openTCP("192.168.10.77", 50241);
    driver_streaming.setExtractLastPacket(true);

    std::cout << "Connected" << std::endl;
    base::samples::Joints current_position = readCurrentPosition(driver_streaming);

    //Start trajectory mode
    msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
    printMotionReply(reply);

    //Check motion ready
    reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
    printMotionReply(reply);

    sleep(1);

    for (int i = 0; i < current_position.size(); i++)
    {
        current_position[i].speed = 0.0;
        current_position[i].acceleration = 0.0;
    }

    //Send first trajectory point (current positon)
    current_position.time = base::Time::fromSeconds(0);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, current_position);
    printMotionReply(reply);

    //Send second trajectory point 
    current_position[0].position = -1.;
    current_position[0].speed = 0.37;
    current_position.time = base::Time::fromSeconds(2);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, current_position);
    printMotionReply(reply);
    
    //Send second trajectory point 
    current_position[0].position = -1.5;
    current_position[0].speed = 0.37;
    current_position.time = base::Time::fromSeconds(4);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 2, current_position);
    printMotionReply(reply);

    //Send second trajectory point 
    current_position[0].position = 1.5;
    current_position[0].speed = 0.0;
    current_position.time = base::Time::fromSeconds(6);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 3, current_position);
    printMotionReply(reply);

    //Stop before motion was complete
    sleep(7);
    reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
    printMotionReply(reply);

    sleep(1);

    current_position = readCurrentPosition(driver_streaming);
    //Send second trajectory point 
    current_position.time = base::Time::fromSeconds(0);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, current_position);
    printMotionReply(reply);
    
    current_position[0].position = 0;
    current_position.time = base::Time::fromSeconds(2);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, current_position);
    printMotionReply(reply);
    sleep(3);
    current_position = readCurrentPosition(driver_streaming);
}
