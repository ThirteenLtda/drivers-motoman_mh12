#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <iostream>
#include <math.h>

namespace motoman_mh12
{


void printJointFeedback(msgs::MotomanJointFeedback &joint_feedback)
{
    std::cout << "############################" << std::endl;
    std::cout << "JOINT FEEDBACK" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        std::cout << "Joint " << i << ": " << joint_feedback.joint_states[i].position * 180 / M_PI << std::endl;
    }
    std::cout << "Timestamp: " << joint_feedback.time << std::endl;
}

void printMotionReply(msgs::MotionReply &reply)
{
    std::cout << "############################" << std::endl;
    std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
    std::cout << "With subcode " << reply.subcode << std::endl;
    std::cout << "sequence_id #" << reply.sequence_id << std::endl;
}

int send_joint_cmd(Driver& driver_ctrl, int sequence_id, base::samples::Joints target_position)
{
    msgs::MotionReply reply;
    int busy = 0;
    // Record start time
    base::Time start = base::Time::now();
    while(true){
        
        reply = driver_ctrl.sendJointTrajPTFullCmd(0, sequence_id, target_position);
        
        if (reply.result == 1)
        {
            busy++;
        }
        else
        {
            base::Time finish = base::Time::now();
            double elapsed = (finish-start).toMilliseconds();
            printMotionReply(reply);
            std::cout << "It took " << elapsed << " ms" << " and " << busy << " tries" << std::endl;
            break;
        } 
    }
}

base::samples::Joints readCurrentPosition(Driver &driver)
{
    while (true)
    {
        base::samples::Joints target_position;
        msgs::MotomanMsgType msg_type = driver.read(base::Time::fromSeconds(2));
        std::cout << ".";
        if (msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
        {
            std::cout << std::endl;
            msgs::MotomanJointFeedback joint_feedback = driver.getJointFeedback();
            printJointFeedback(joint_feedback);
            target_position.elements = joint_feedback.joint_states;
            target_position.time = joint_feedback.time;
            return target_position;
        }
    }
}
}



