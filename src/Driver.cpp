#include "Driver.hpp"
#include "motoman_mh12Constants.hpp"
#include "motoman_mh12Msgs.hpp"
#include <iostream>
#include <string.h>
#include <base/Time.hpp>
#include <base/Timeout.hpp>

using namespace motoman_mh12;

static const int LENGTH_UNKNOWN = -1;

Driver::Driver()
: iodrivers_base::Driver(10*MotomanMsgTypes::MOTOMAN_MAX_PKT_SIZE) 
{
    buffer.resize(10*MotomanMsgTypes::MOTOMAN_MAX_PKT_SIZE);
}
// The count above is the maximum packet size

MotomanMsgTypes::MotomanMsgType Driver::read()
{
    int packet_size = readPacket(&buffer[0], buffer.size());
    return parsePacket(&buffer[0], packet_size);
}

/*
* Returns the expected packet size given a message type, if there is
* no match, it return -1 to move the buffer pointer
* @param msg_type Message type from the header
*/
int Driver::returnMsgSize(int msg_type) const
{
    switch(msg_type)
    {
        case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
            return MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE 
			+ MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE;
        case MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK:
            return MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE 
			+ MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK_SIZE;
        case MotomanMsgTypes::MOTOMAN_MOTION_REPLY:
            return MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE 
			+ MotomanMsgTypes::MOTOMAN_MOTION_REPLY_SIZE;
        case MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY:
            return MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE 
			+ MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY_SIZE;
        case MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO_REPLY:
            return MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE 
			+ MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO_REPLY_SIZE;
        default:
            return LENGTH_UNKNOWN;
    }
	}


int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    
    if(buffer_size < sizeof(int32_t)*2)
        return 0;
    
    int length = buffer_as_int32[0];
    int msg_type = buffer_as_int32[1];
    int expected_length = returnMsgSize(msg_type);
    
    if(length != expected_length)
        return LENGTH_UNKNOWN;
    
    if(buffer_size>=length)
        return expected_length;
    else
        return 0;
    
}

MotomanMsgTypes::MotomanMsgType Driver::parsePacket(uint8_t const* buffer, size_t size)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    switch(buffer_as_int32[1])
    {
        case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
            status  = parseReadStatus(buffer,size);
            return MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
        case MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK:
            joint_feedback = parseJointFeedback(buffer);
            return MotomanMsgTypes::MOTOMAN_JOINT_FEEDBACK;
    }
}

/*
* Interpret the tristate 1 = TRUE, 0 = FALSE and -1=Unknown as a boolend and
* it was adopted the convention to always throw if any unknown state is received
*/
static bool interpret_tristate(int32_t flag)
{
    if(flag == -1)
        throw std::runtime_error("Unknown state received.");
    return bool(flag);
}

msgs::MotomanStatus Driver::parseReadStatus(uint8_t const* buffer, size_t size) const
{
    msgs::StatusMsg const& msg = *reinterpret_cast<msgs::StatusMsg const*>(buffer);
    msgs::MotomanStatus motoman_status;
    motoman_status.drives_powered = interpret_tristate(msg.drives_powered);
    motoman_status.e_stopped = interpret_tristate(msg.e_stopped);
    // it was adopted the convention to always throw if any unknown state is received
    if(msg.error_code == -1)
        throw std::runtime_error("alarms in unkown state");
    else
        motoman_status.error_code = int(msg.error_code);
    motoman_status.ln_error = interpret_tristate(msg.ln_error);
    motoman_status.ln_motion = interpret_tristate(msg.ln_motion);
    motoman_status.mode = interpret_tristate(msg.mode);
    motoman_status.motion_possible = interpret_tristate(msg.motion_possible);
    return motoman_status;
}

msgs::MotomanJointFeedback Driver::parseJointFeedback(uint8_t const* buffer) const
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[4*4]);
    msgs::MotomanJointFeedback parsed_joint_feedback;
    parsed_joint_feedback.robot_id = int(buffer_as_int32[1]);
    parsed_joint_feedback.valid_field = int(buffer_as_int32[3]);
    //The MotoROS driver send only the position, so the bit masking of the valid fields
    //must be always 2.
    if(parsed_joint_feedback.valid_field !=2)
        throw std::runtime_error("Bit-masking of valid fields inconsistent");
    
    float const* buffer_as_float = reinterpret_cast<float const*>(&buffer_as_int32[2]);
    parsed_joint_feedback.time.fromSeconds(buffer_as_float[0]);
    for(int i = 0; i<10; i++)
    {
        base::JointState joint_state;
        joint_state.position = double(buffer_as_float[1+i]);
        parsed_joint_feedback.joint_states.push_back(joint_state);
    }
    
    return parsed_joint_feedback;
}

msgs::MotionReply Driver::parseMotionReply(uint8_t const* buffer)
{
    msgs::MotionReplyMsg const& msg = *reinterpret_cast<msgs::MotionReplyMsg const*>(buffer);
    msgs::MotionReply motion_reply;
    motion_reply.robot_id = msg.robot_id;
    motion_reply.sequence = msg.sequence;
    motion_reply.command = msg.command;
    motion_reply.result = msg.result;
    return motion_reply;
    
}

void Driver::sendJointTrajPTFullCmd(int robot_id, int sequence, base::Time timestamp,
                                    std::vector<base::JointState> const& joint_states)
{
    msgs::JointTrajPTFullMsg joint_traj_cmd;
    joint_traj_cmd.prefix.length = 46; 
    joint_traj_cmd.prefix.msg_type = MotomanMsgTypes::MOTOMAN_JOINT_TRAJ_PT_FULL;
    joint_traj_cmd.robot_id = int32_t(robot_id);
    joint_traj_cmd.sequence = int32_t(sequence);
    joint_traj_cmd.time = timestamp.toSeconds();
    for(size_t i=0; i<joint_states.size();i++)
    {
        joint_traj_cmd.positions[i] =  joint_states[i].position;
        joint_traj_cmd.velocities[i] = joint_states[i].speed;
        joint_traj_cmd.accelerations[i] = joint_states[i].acceleration;
    }
    
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&joint_traj_cmd);
    writePacket(buffer, joint_traj_cmd.prefix.length + 4);
}

void Driver::waitForReply(base::Time const& timeout, int32_t msg_type)
{
    base::Timeout deadline = base::Timeout(timeout);
    
    while(!deadline.elapsed())
    {
        int packet_size = readPacket(&buffer[0], buffer.size(), deadline.timeLeft());
        int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[0]);
        if(buffer_as_int32[1]== msg_type)
            return;
    }
    throw std::runtime_error("timeout reached and no reply received");
}

msgs::MotionReply Driver::sendMotionCtrl(int robot_id, int sequence, int cmd)
{
    msgs::MotionCtrlMsg motion_ctrl;
    motion_ctrl.prefix.length = 52;
    motion_ctrl.prefix.msg_type = MotomanMsgTypes::MOTOMAN_MOTION_CTRL;
    motion_ctrl.robot_id = int32_t(robot_id);
    motion_ctrl.sequence = int32_t(sequence);
    motion_ctrl.cmd = int32_t(cmd);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&motion_ctrl);
    writePacket(buffer, motion_ctrl.prefix.length + 4);
    return readMotionCtrlReply(base::Time::fromSeconds(0.1));
}

msgs::MotionReply Driver::readMotionCtrlReply(const base::Time& timeout)
{
    waitForReply(timeout,MotomanMsgTypes::MOTOMAN_MOTION_REPLY);
    return parseMotionReply(&buffer[0]);
}

void Driver::parseReadSingleIOReply(uint8_t const* buffer)
{
    msgs::ReadSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::ReadSingleIoReplyMsg const*>(buffer);
}

void Driver::sendReadSingleIO(int IOaddress)
{
    msgs::ReadSingleIoMsg read_single_io;
    read_single_io.prefix.length = 8;
    read_single_io.prefix.msg_type = MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO;
    read_single_io.address = IOaddress;
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&read_single_io);
    writePacket(buffer,read_single_io.prefix.length + 4);
    readSingleIOReply(base::Time::fromSeconds(0.1));
}

void Driver::readSingleIOReply(const base::Time& timeout)
{
    waitForReply(timeout, MotomanMsgTypes::MOTOMAN_READ_SINGLE_IO_REPLY);
    parseReadSingleIOReply(&buffer[0]);
}

bool Driver::sendWriteSingleIo(int IOaddress, int value)
{
    msgs::WriteSingleIoMsg write_single_io;
    write_single_io.prefix.length = 0;
    write_single_io.prefix.msg_type = MotomanMsgTypes:: MOTOMAN_WRITE_SINGLE_IO;
    write_single_io.io_address = IOaddress;
    write_single_io.value = value;
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&write_single_io);
    writePacket(buffer,write_single_io.prefix.length + 4);
    return readWriteSingleIO(base::Time::fromSeconds(0.1));
};

bool Driver::readWriteSingleIO(const base::Time& timeout)
{
    waitForReply(timeout, MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO_REPLY);
    return parseWriteSingleIOReply(&buffer[0]);
}

bool Driver::parseWriteSingleIOReply(uint8_t const* buffer) const
{
    msgs::WriteSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::WriteSingleIoReplyMsg const*>(buffer);
    if(msg.result_code == write_single_io::SUCCESS)
        return true;
    else
        return false;
}
