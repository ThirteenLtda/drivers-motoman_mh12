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
           std::cout << "Robot id: " << joint_feedback.robot_id << std::endl;
           std::cout << "Joint 0: " << joint_feedback.joint_states[0].position << std::endl;
           std::cout << "Joint 1: " << joint_feedback.joint_states[1].position << std::endl;
           std::cout << "Joint 2: " << joint_feedback.joint_states[2].position << std::endl;
           std::cout << "Joint 3: " << joint_feedback.joint_states[3].position << std::endl;
           std::cout << "Joint 4: " << joint_feedback.joint_states[4].position << std::endl;
           std::cout << "Joint 5: " << joint_feedback.joint_states[5].position << std::endl;
           std::cout << "Joint 6: " << joint_feedback.joint_states[6].position << std::endl;
           std::cout << "Joint 7: " << joint_feedback.joint_states[7].position << std::endl;
           std::cout << "Joint 8: " << joint_feedback.joint_states[8].position << std::endl;
           std::cout << "Joint 9: " << joint_feedback.joint_states[9].position << std::endl;
           std::cout << "TIME: " << joint_feedback.time.toSeconds() << std::endl;
           break;
       }
  }
       
  msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
  
  for(int i=0; i<current_position.size();i++)
  {
      current_position[i].speed = 0.0;
      current_position[i].acceleration = 0.0;
  }
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, base::Time::fromSeconds(0), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
  
  current_position[0].position = 1.57;
  
  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 1, base::Time::fromSeconds(5), current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;
 
  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
  std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
        
}
  
