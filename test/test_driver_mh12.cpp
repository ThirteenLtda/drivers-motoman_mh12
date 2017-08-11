#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <iostream>

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
       msgs::MotomanMsgType msg_type =  driver_streaming.read();
       if(msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
       {
            msgs::MotomanJointFeedback joint_feedback = driver_streaming.getJointFeedback();
            current_position = joint_feedback.joint_states;
            std::cout << "Current Position aquired" << std::endl;
            break;
       }
  }
       
  msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, base::Time::now(), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
  
  current_position[3].position = 0.9;
  current_position[3].speed = 0.2;
  current_position[3].acceleration = 50;
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, base::Time::now(), current_position);
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 2, base::Time::now(), current_position);
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 3, base::Time::now(), current_position);
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 4, base::Time::now(), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
  
  
  current_position[3].position = 0.9;
  current_position[3].speed = 0.0;
  current_position[3].acceleration = 0;
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 5, base::Time::now(), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
 
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
        
}
  
