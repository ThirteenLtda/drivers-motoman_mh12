#include "Driver.hpp"
#include "motoman_mh12Constants.hpp"
#include "motoman_mh12Msgs.hpp"
#include <iostream>
#include <string.h>

using namespace motoman_mh12;
Driver::Driver()
: iodrivers_base::Driver(10000) {}
// The count above is the maximum packet size

int Driver::read()
{
    uint8_t buffer[10000];
    int packet_size = readPacket(buffer, 10000);
    return parsePacket(buffer, packet_size);
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
    
    if (expected_length == -1)
        return expected_length;
    
    if(length>=expected_length)
        return expected_length;
    else
        return 0;
    
}

int Driver::parsePacket(uint8_t const* buffer, size_t size)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    switch(buffer_as_int32[1])
    {
        case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
            parseReadStatus(buffer,size);
            return MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
        case MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK:
            parseJointFeedback(buffer,size);
            return MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK;
    }
}


static bool interpret_tristate(int32_t flag)
{
    if(flag == -1)
        throw std::runtime_error("Unknown state received.");
    return bool(flag);
}

void Driver::parseReadStatus(uint8_t const* buffer, size_t size)
{
    msgs::StatusMsg const& msg = *reinterpret_cast<msgs::StatusMsg const*>(buffer);
    msgs::MotomanStatus status;
    status.drives_powered = interpret_tristate(msg.drives_powered);
    status.e_stopped = interpret_tristate(msg.e_stopped);
    if(msg.error_code == -1)
        throw std::runtime_error("alarms in unkown state");
    else
        status.error_code = int(msg.error_code);
    status.ln_error = interpret_tristate(msg.ln_error);
    status.ln_motion = interpret_tristate(msg.ln_motion);
    status.mode = interpret_tristate(msg.mode);
    status.motion_possible = interpret_tristate(msg.motion_possible);
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
    joint_feedback.time.fromSeconds(buffer_as_float[0]);
    for(int i = 0; i<10; i++)
    {
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