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
            parseJointFeedback(buffer);
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

void Driver::parseJointFeedback(uint8_t const* buffer)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[4*4]);
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

void Driver::parseMotionReply(uint8_t const* buffer)
{
    msgs::MotionReplyMsg const& msg = *reinterpret_cast<msgs::MotionReplyMsg const*>(buffer);
}

void Driver::parseReadSingleIOReply(uint8_t const* buffer, size_t size)
{
    msgs::ReadSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::ReadSingleIoReplyMsg const*>(buffer);
}



void Driver::sendJointTrajPTFullCmd(int robot_id, int sequence, base::Time timestamp,
                                    std::vector<base::JointState> joint_states)
{
    msgs::JointTrajPTFullMsg joint_traj_cmd;
    joint_traj_cmd.prefix.length = 46; 
    joint_traj_cmd.prefix.msg_type = MotomanMsgTypes::MOTOMAN_JOINT_TRAJ_PT_FULL;
    joint_traj_cmd.robot_id = int32_t(robot_id);
    joint_traj_cmd.sequence = int32_t(sequence);
    joint_traj_cmd.time = timestamp.toSeconds();
    for(int i=0; i<joint_states.size();i++)
    {
        joint_traj_cmd.positions[i] =  joint_states[i].position;
        joint_traj_cmd.velocities[i] = joint_states[i].speed;
        joint_traj_cmd.accelerations[i] = joint_states[i].acceleration;
    }
    
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&joint_traj_cmd);
    writePacket(buffer, joint_traj_cmd.prefix.length + 4);
    readJointFeedback(base::Time::fromSeconds(0.1));
}

void Driver::readJointFeedback(base::Time const& timeout)
{
    base::Timeout deadline = base::Timeout(timeout);
    
    while(deadline.timeLeft().toMicroseconds() > .0)
    {
        int packet_size = readPacket(&buffer[0], 10000, deadline.timeLeft());
        int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[0]);
        if(buffer_as_int32[1]!= MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK)
            continue;
        else
            parseJointFeedback(&buffer[0]);
            return;
    }
}



void Driver::sendMotionCtrl(int robot_id, int sequence, int cmd)
{
    msgs::MotionCtrlMsg motion_ctrl;
    motion_ctrl.prefix.length = 52;
    motion_ctrl.prefix.msg_type = MotomanMsgTypes::MOTOMAN_MOTION_CTRL;
    motion_ctrl.robot_id = int32_t(robot_id);
    motion_ctrl.sequence = int32_t(sequence);
    motion_ctrl.cmd = int32_t(cmd);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&motion_ctrl);
    writePacket(buffer, motion_ctrl.prefix.length + 4);
    readMotionCtrlReply(base::Time::fromSeconds(0.1));
}

void Driver::readMotionCtrlReply(const base::Time& timeout)
{
    base::Timeout deadline = base::Timeout(timeout);
    
    while(deadline.timeLeft().toMicroseconds() > .0)
    {
        int packet_size = readPacket(&buffer[0], 10000, deadline.timeLeft());
        int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[0]);
        if(buffer_as_int32[1]!= MotomanMsgTypes::MOTOMAN_MOTION_REPLY)
            continue;
        else
            parseMotionReply(&buffer[0]);
            return;
    }
}
