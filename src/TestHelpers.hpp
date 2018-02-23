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
}

base::samples::Joints readCurrentPosition(Driver &driver)
{
    while (true)
    {
        base::samples::Joints current_position;
        msgs::MotomanMsgType msg_type = driver.read();
        std::cout << ".";
        if (msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
        {
            std::cout << std::endl;
            msgs::MotomanJointFeedback joint_feedback = driver.getJointFeedback();
            printJointFeedback(joint_feedback);
            current_position.elements = joint_feedback.joint_states;
            current_position.time = joint_feedback.time;
            return current_position;
        }
    }
}
}



