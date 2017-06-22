#include "Driver.hpp"
#include "motoman_mh12Constants.hpp"
#include "motoman_mh12Msgs.hpp"
#include <iostream>
#include <string.h>

using namespace motoman_mh12;
Driver::Driver()
: iodrivers_base::Driver(10000) {}
// The count above is the maximum packet size

void Driver::read()
{
    uint8_t buffer[10000];
    int packet_size = readPacket(buffer, 10000);
    parsePacket(buffer, packet_size);
}

int Driver::returnMsgSize(int msg_type) const
{
    switch(msg_type)
    {
        case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
            return MotomanMsgTypes::MOTOMAN_HEADER_MSG_SIZE + MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE;
        case MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK:
            return MotomanMsgTypes::MOTOMAN_HEADER_MSG_SIZE + MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK_SIZE;
        case MotomanMsgTypes::MOTOMAN_MOTION_REPLY:
            return MotomanMsgTypes::MOTOMAN_HEADER_MSG_SIZE + MotomanMsgTypes::MOTOMAN_MOTION_REPLY_SIZE;
        case MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY:
            return MotomanMsgTypes::MOTOMAN_HEADER_MSG_SIZE + MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY_SIZE;
        case MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO_REPLY:
            return MotomanMsgTypes::MOTOMAN_HEADER_MSG_SIZE + MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO_REPLY_SIZE;
        default:
            return -1;
    }
}


int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    int length = buffer_as_int32[0];
    int msg_type = buffer_as_int32[1];
    int expected_length = returnMsgSize(msg_type);
    if(length>=expected_length)
        return expected_length;
    else
        return 0;
    
}

void Driver::parsePacket(uint8_t const* buffer, size_t size)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    switch(buffer_as_int32[1])
    {
        case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
            parseReadStatus(buffer,size);
        case MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK:
            parseJointFeedback(buffer,size);
        case MotomanMsgTypes::MOTOMAN_MOTION_REPLY:
            parseMotionReply(buffer,size);
        case MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY:
            parseReadSingleIOReply(buffer,size);
    }
}

void Driver::parseReadStatus(uint8_t const* buffer, size_t size)
{
    msgs::StatusMsg const& msg = *reinterpret_cast<msgs::StatusMsg const*>(buffer);
    msgs::MotomanStatus status;
    if (msg.drives_powered == -1)
        throw std::runtime_error("robot drivers in unknown state");
    else 
        status.drives_powered = bool(msg.drives_powered);
    if(msg.e_stopped == -1)
        throw std::runtime_error("e-stop controller in unkown state");
    else
        status.e_stopped = bool(msg.e_stopped);
    if(msg.error_code == -1)
        throw std::runtime_error("alarms in unkown state");
    else
        status.error_code = int(msg.error_code);
    if(msg.ln_error == -1)
        throw std::runtime_error("alarms in unkown state");
    else 
        status.ln_error = bool(msg.ln_error);
    if(msg.ln_motion == -1)
        throw std::runtime_error("controller in unkown state");
    else
        status.ln_motion = bool(msg.ln_motion);
    if(msg.mode == -1)
        throw std::runtime_error("controller mode is unknown");
    else 
        status.mode = bool(msg.mode);
    if(msg.motion_possible == -1)
        throw std::runtime_error("ROCK communication state is unknown");
    else
        status.motion_possible = bool(msg.motion_possible);
}

void Driver::parseJointFeedback(uint8_t const* buffer, size_t size)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[4*4]);
    msgs::MotomanJointFeedback joint_feedback;
    joint_feedback.robot_id = int(buffer_as_int32[1]);
    joint_feedback.valid_field = int(buffer_as_int32[3]);
    if(joint_feedback.valid_field !=2)
        throw std::runtime_error("Bit-masking of valid field inconsistent");
    
    float const* buffer_as_float = reinterpret_cast<float const*>(&buffer_as_int32[2]);
    for(int i = 0; i<10; i++)
    {
        joint_feedback.time.fromSeconds(buffer_as_float[0]);
        base::JointState joint_state;
        joint_state.position = double(buffer_as_float[1+i]);
        joint_feedback.joint_states.push_back(joint_state);
    }
    
}

void Driver::parseMotionReply(uint8_t const* buffer, size_t size)
{
    msgs::MotionReplyMsg const& msg = *reinterpret_cast<msgs::MotionReplyMsg const*>(buffer);
}

void Driver::parseReadSingleIOReply(uint8_t const* buffer, size_t size)
{
    msgs::ReadSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::ReadSingleIoReplyMsg const*>(buffer);
}