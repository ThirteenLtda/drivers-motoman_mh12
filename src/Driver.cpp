#include "Driver.hpp"
#include <iostream>
#include <string.h>
#include <base/Time.hpp>
#include <base/Timeout.hpp>

using namespace motoman_mh12;

Driver::Driver()
: iodrivers_base::Driver(10*msgs::MOTOMAN_MAX_PKT_SIZE) 
{
    buffer.resize(10*msgs::MOTOMAN_MAX_PKT_SIZE);
}
// The count above is the maximum packet size

msgs::MotomanMsgType Driver::read(base::Time timeout)
{
    int packet_size = readPacket(&buffer[0], buffer.size(),timeout);
    return parsePacket(&buffer[0], packet_size);
}


int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    
    if(buffer_size < sizeof(int32_t)*2)
        return 0;
    
    int length = buffer_as_int32[0];
    int msg_type = buffer_as_int32[1];
    int expected_length = msgs::returnMsgSize(msg_type);
    
    if(length != expected_length)
        return -1;
    
    if(buffer_size>=length)
        return expected_length;
    else
        return 0;
    
}

msgs::MotomanMsgType Driver::parsePacket(uint8_t const* buffer, size_t size)
{
    int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
    switch(buffer_as_int32[1])
    {
        case msgs::MOTOMAN_ROBOT_STATUS:
            status  = parseReadStatus(buffer,size);
            return msgs::MOTOMAN_ROBOT_STATUS;
        case msgs::MOTOMAN_JOINT_FEEDBACK:
            joint_feedback = parseJointFeedback(buffer);
            return msgs::MOTOMAN_JOINT_FEEDBACK;
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
    msgs::JointFeedbackMsg const& msg = *reinterpret_cast<msgs::JointFeedbackMsg const*>(buffer);
    msgs::MotomanJointFeedback parsed_joint_feedback;
    parsed_joint_feedback.robot_id = msg.robot_id;
    parsed_joint_feedback.valid_field = msg.valid_field;
    //The MotoROS driver sends position and velocity, so the bit masking of the valid fields
    //must be always 6
    if(parsed_joint_feedback.valid_field !=6)
        throw std::runtime_error("Bit-masking of valid fields inconsistent");
    
    parsed_joint_feedback.time.fromSeconds(msg.time);
    for(int i = 0; i<10; i++)
    {
        base::JointState joint_state;
        joint_state.position = double(msg.positions[i]);
        parsed_joint_feedback.joint_states.push_back(joint_state);
    }
    
    return parsed_joint_feedback;
}

msgs::MotionReply Driver::parseMotionReply(uint8_t const* buffer)
{
    msgs::MotionReplyMsg const& msg = *reinterpret_cast<msgs::MotionReplyMsg const*>(buffer);
    msgs::MotionReply motion_reply;
    motion_reply.robot_id = msg.robot_id;
    motion_reply.sequence_id = msg.sequence_id;
    motion_reply.command = msg.command;
    motion_reply.result = msg.result;
    motion_reply.subcode = msg.subcode;
    return motion_reply;
}

float removeNaNs(float a)
{
    if(a != a)
    {
        return 0.0;
    }
    else
        return a;
}

msgs::MotionReply Driver::sendJointTrajPTFullCmd(int robot_id, int sequence_id, base::samples::Joints const& joints_samples)
{
    msgs::JointTrajPTFullMsg joint_traj_cmd;
    joint_traj_cmd.prefix.length = msgs::MOTOMAN_JOINT_TRAJ_PT_FULL_SIZE; 
    joint_traj_cmd.prefix.msg_type = msgs::MOTOMAN_JOINT_TRAJ_PT_FULL;
    joint_traj_cmd.robot_id = int32_t(robot_id);
    joint_traj_cmd.sequence_id = int32_t(sequence_id);
    joint_traj_cmd.time = joints_samples.time.toSeconds();
    for(size_t i=0; i<joints_samples.size();i++)
    {
        joint_traj_cmd.positions[i] =  removeNaNs(joints_samples[i].position);
        joint_traj_cmd.velocities[i] = removeNaNs(joints_samples[i].speed);
        joint_traj_cmd.accelerations[i] = removeNaNs(joints_samples[i].acceleration);
    
    }
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&joint_traj_cmd);
    writePacket(buffer, joint_traj_cmd.prefix.length + 4);
    return readMotionCtrlReply(base::Time::fromSeconds(1));
}

void Driver::waitForReply(base::Time const& timeout, int32_t msg_type)
{
    base::Timeout deadline(timeout);
    
    while(!deadline.elapsed())
    {
        int packet_size = readPacket(&buffer[0], buffer.size(), deadline.timeLeft());
        int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(&buffer[0]);
        if(buffer_as_int32[1] == msg_type)
            return;
    }
    throw std::runtime_error("timeout reached and no reply received");
}

msgs::MotionReply Driver::sendJointPosition(int sequence_id, std::vector<float> joints_positions)
{
    msgs::JointPositionMsg joint_position_msg;
    joint_position_msg.sequence_id = int32_t(sequence_id);
    for(size_t i=0; i<joints_positions.size();i++)
        joint_position_msg.joints[i] =  removeNaNs(joints_positions[i]);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&joint_position_msg);
    writePacket(buffer, joint_position_msg.prefix.length + 4);
    return readJointPositionReply(base::Time::fromSeconds(1));
}

msgs::MotionReply Driver::sendMotionCtrl(int robot_id, int sequence_id, int cmd)
{
    msgs::MotionCtrlMsg motion_ctrl(robot_id, sequence_id, cmd);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&motion_ctrl);
    writePacket(buffer, motion_ctrl.prefix.length + 4);
    return readMotionCtrlReply(base::Time::fromSeconds(1));
}

msgs::MotionReply Driver::readJointPositionReply(const base::Time& timeout)
{
    //Expect Motion Reply, but unsure
    waitForReply(timeout,msgs::MOTOMAN_MOTION_REPLY);
    return parseMotionReply(&buffer[0]);
}

msgs::MotionReply Driver::readMotionCtrlReply(const base::Time& timeout)
{
    waitForReply(timeout,msgs::MOTOMAN_MOTION_REPLY);
    return parseMotionReply(&buffer[0]);
}

msgs::ReadSingleIo Driver::parseReadSingleIOReply(uint8_t const* buffer)
{
    msgs::ReadSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::ReadSingleIoReplyMsg const*>(buffer);
    msgs::ReadSingleIo single_io;
    single_io.value = msg.value;
    single_io.result = msg.result_code;
    return single_io;
}

msgs::ReadSingleIo Driver::queryReadSingleIO(int32_t IOaddress)
{
    msgs::ReadSingleIoMsg read_single_io(IOaddress);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&read_single_io);
    writePacket(buffer, read_single_io.prefix.length +4);
    return readSingleIOReply(base::Time::fromSeconds(1));
}

msgs::ReadSingleIo Driver::readSingleIOReply(const base::Time& timeout)
{
    waitForReply(timeout, msgs::MOTOMAN_READ_SINGLE_IO_REPLY);
    return parseReadSingleIOReply(&buffer[0]);
}

bool Driver::sendWriteSingleIo(int IOaddress, int value)
{
    msgs::WriteSingleIoMsg write_single_io(IOaddress, value);
    uint8_t const* buffer = reinterpret_cast<uint8_t const*>(&write_single_io);
    writePacket(buffer,write_single_io.prefix.length + 4);
    waitForReply(base::Time::fromSeconds(0.1), msgs::MOTOMAN_WRITE_SINGLE_IO_REPLY);
    return parseWriteSingleIOReply(&buffer[0]);
}

bool Driver::parseWriteSingleIOReply(uint8_t const* buffer) const
{
    msgs::WriteSingleIoReplyMsg const& msg = *reinterpret_cast<msgs::WriteSingleIoReplyMsg const*>(buffer);
    return (msg.result_code == msgs::write_single_io::SUCCESS);
}

msgs::MotomanStatus Driver::getRobotStatus()
{
    return status;
}

msgs::MotomanJointFeedback Driver::getJointFeedback()
{
    return joint_feedback;
}
