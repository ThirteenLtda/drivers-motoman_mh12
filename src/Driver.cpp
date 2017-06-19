#include "Driver.hpp"
#include "motoman_mh12Constants.hpp"
#include "motoman_mh12Msgs.hpp"
#include <iostream>

using namespace motoman_mh12;
Driver::Driver()
: iodrivers_base::Driver(10000) {}
// The count above is the maximum packet size

void Driver::open(std::string const& uri)
{
  openURI(uri);
}

void Driver::read()
{
  uint8_t buffer[10000];
  int packet_size = readPacket(buffer, 10000);
  parsePacket(buffer, packet_size);
}

// Reimplement close() only if you have specific things to do
// with your device before closing the I/O
void Driver::close()
{
  iodrivers_base::Driver::close();
}

int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
  int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
  switch(buffer_as_int32[0])
  {
    case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
      if(buffer_size< MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE)
	return 0;
      else
	return MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE;
    
    default:
      return -1;
  }
}

void Driver::parsePacket(uint8_t const* buffer, size_t size)
{
  int32_t const* buffer_as_int32 = reinterpret_cast<int32_t const*>(buffer);
  switch(buffer_as_int32[0])
  {
    case MotomanMsgTypes::MOTOMAN_ROBOT_STATUS:
      parseReadStatus(buffer,size);
    case MotomanMsgTypes::MOTOMAN_JOINT_TRAJ_PT_FULL:
      parseJointTrajPtFull(buffer,size);
  }
}

void Driver::parseReadStatus(uint8_t const* buffer, size_t size)
{
  msgs::StatusMessage const& msg = *reinterpret_cast<msgs::StatusMessage const*>(buffer);
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