#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <motoman_mh12/TestHelpers.hpp>
#include <iostream>
#include <math.h>

using namespace motoman_mh12;
/**
 * Test the behavior of the robot when a 
 * stop command is sent in the middle of the trajectory
 * 
 * Result: the robot stops as soon as possible ("immediately")
 * and doesn't activate the emmergency brakes. It can receive
 * a new trajectory after the brake normally. It is mandatory
 * to restart the sequence count, i.e begin a NEW trajectory, 
 * otherwise it can lead to extremely wrong trajectories.
 * */
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
    current_position[0].position = 1.57;
    current_position.time = base::Time::fromSeconds(5);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, current_position);
    printMotionReply(reply);

    //Stop before motion was complete
    sleep(1);
    reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
    printMotionReply(reply);

    current_position = readCurrentPosition(driver_streaming);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, current_position);
    sleep(8);
}
