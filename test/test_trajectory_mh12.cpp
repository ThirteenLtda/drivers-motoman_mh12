#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <iostream>
#include <math.h>

using namespace motoman_mh12;


int main(int argc, char **argv)
{
  Driver driver_ctrl;
  driver_ctrl.openTCP("192.168.10.77", 50240);
  
  Driver driver_streaming;
  driver_streaming.openTCP("192.168.10.77", 50241);
  
  std::vector<base::JointState> current_position;
  while(true)
  {  
       std::cout << "Streamer Msg received" << std::endl;      
       msgs::MotomanMsgType msg_type =  driver_streaming.read(base::Time::fromMicroseconds(500000));
       if(msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
       {
           msgs::MotomanJointFeedback joint_feedback = driver_streaming.getJointFeedback();
           current_position = joint_feedback.joint_states;
           std::cout << "Current Position aquired" << std::endl;
           std::cout << "Robot id: " << joint_feedback.robot_id << std::endl;
           for(size_t i = 0; i<joint_feedback.joint_states.size(); i++)
              std::cout << "Joint "<< i <<": " << joint_feedback.joint_states[i].position*180/M_PI << std::endl;
           std::cout << "TIME: " << joint_feedback.time.toSeconds() << std::endl;
           break;
       }
  }
       
  msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  
  for(int i=0; i<current_position.size();i++)
  {
      current_position[i].speed = 0.0;
      current_position[i].acceleration = 0.0;
  }
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, base::Time::fromSeconds(0), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
  
  current_position[0].position = 0.9;
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, base::Time::fromSeconds(5), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
 
  sleep(2);
  
  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
  //std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
  //std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
        
}
  
