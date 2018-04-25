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


    //Test the behavior of sending multiple start and stop commands

    for(int i=0; i<10; i++)
    {
        msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(
            0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
        if(reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
            printMotionReply(reply);
    }
    for(int i=0; i<10; i++)
    {
        msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(
            0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
        if(reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
            printMotionReply(reply);
    }
    for(int i=0; i<10; i++)
    {
        msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(
            0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
        if(reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
            printMotionReply(reply);
    }
}
