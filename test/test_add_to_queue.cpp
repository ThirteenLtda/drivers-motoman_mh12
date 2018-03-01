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
    double inc = 0.01;
    double angle_limit = 45;
    int joint = 0;
    //send sequence of small incremental movement
    for (int i = 1; i < angle_limit; i++)
    {
        int busy = 0;
        base::Time start = base::Time::now();
        // Record start time
        while(true){
            current_position.time = base::Time::fromSeconds(inc*(i));
            current_position[joint].position = base::Angle::deg2Rad(i);
            current_position[joint].speed = base::Angle::deg2Rad(1./inc);
            if(i == int(angle_limit/2))
            {
                reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
                printMotionReply(reply);
                sleep(1); 
                reply = driver_ctrl.sendJointTrajPTFullCmd(0, i, current_position);
            }
            else
                reply = driver_ctrl.sendJointTrajPTFullCmd(0, i, current_position);
            
            if (reply.result == 1)
            {
                busy++;
            }
            else
            {
                base::Time finish = base::Time::now();
                double elapsed = (finish-start).toMilliseconds();
                std::cout << "It took " << elapsed << " ms" << " and " << busy << " tries" << std::endl;
                printMotionReply(reply);
                reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);
                printMotionReply(reply);
                break;
            } 
        }
        // Record end time
    }

    sleep(1);
    reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
    printMotionReply(reply);

    sleep(inc*angle_limit);
}

int main(int argc, char **argv)
{
    it_tests_if_sending_small_steps_oferflow_the_controller();
}
