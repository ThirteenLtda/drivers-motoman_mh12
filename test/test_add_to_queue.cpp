#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <base/Angle.hpp>
#include <motoman_mh12/TestHelpers.hpp>
#include <iostream>
#include <math.h>

using namespace motoman_mh12;

/**
 * Given the documentation:
 * In the case of motion message that adds a trajectory point used to generate incremental
 * motion in the queue, if the queue is full the reply could be delayed. This is undesirable
 * because it would prevent other message such as stop motion to come in. So in order to
 * prevent this situation, the message is parsed and its data validated, the point data is then
 * temporarily copied to be processed by the AddToIncQueueProcess background task. So a
 * success reply on such instruction only indicates that the message was accepted and not that
 * the motion was completely processed into incremental moves. If the background task is
 * already processing a point, a busy reply will be return to the client and the client will have to
 * resend the message again.
 * 
 * Add to Inc Move Queue Task
 * 
 * The AddToIncQueueProcess task (figure 6) is a background tasks that take a motion 
 * message’s trajectory data and breaks it down to incremental motion segments that matches
 * the controller interpolation period. In the case of multiple motion server connections, you
 * would have one AddToIncMoveQueue task for each motion server task.
 * */



void it_tests_if_sending_small_steps_oferflow_the_controller()
{
    Driver driver_ctrl;
    driver_ctrl.openTCP("192.168.10.77", 50240);

    Driver driver_streaming;
    driver_streaming.openTCP("192.168.10.77", 50241);
    driver_streaming.setExtractLastPacket(true);

    std::cout << "Connected" << std::endl;
    base::samples::Joints current_position = readCurrentPosition(driver_streaming);
    for (int i = 0; i < current_position.size(); i++)
    {
        current_position[i].speed = 0.0;
        current_position[i].acceleration = 0.0;
    }

    //Start trajectory mode
    msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
    printMotionReply(reply);
    sleep(1);

    //Check motion ready
    reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
    printMotionReply(reply);

    //Send first trajectory point (current positon)
    current_position.time = base::Time::fromSeconds(0);
    reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, current_position);
    printMotionReply(reply);
    //send sequence of small incremental movements
    for (int i = 1; i < 300; i++)
    {
        current_position.time = base::Time::fromSeconds(0.01*i);
        current_position[5].position = base::Angle::deg2Rad(i);
        current_position[5].speed = base::Angle::deg2Rad(1)/(0.01);
        reply = driver_ctrl.sendJointTrajPTFullCmd(0, i, current_position);
        printMotionReply(reply);
    }
}

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
