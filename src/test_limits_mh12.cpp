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

  std::cout << "Connected" << std::endl;
  base::samples::Joints current_position;
  while(true)
  {
       msgs::MotomanMsgType msg_type =  driver_streaming.read(base::Time::fromSeconds(2));
       std::cout << "Streamer Msg received" << std::endl;
       if(msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
       {
           msgs::MotomanJointFeedback joint_feedback = driver_streaming.getJointFeedback();
           current_position.elements = joint_feedback.joint_states;
           std::cout << "Current Position aquired" << std::endl;
           std::cout << "Robot id: " << joint_feedback.robot_id << std::endl;
           for(size_t i = 0; i<joint_feedback.joint_states.size(); i++)
              std::cout << "Joint "<< i <<": " << joint_feedback.joint_states[i].position*180/M_PI << std::endl;
           std::cout << "TIME: " << joint_feedback.time.toSeconds() << std::endl;
           break;
       }
  }

  // Fill speed and acceleration
  for(int i=0; i<current_position.size();i++)
  {
      current_position[i].speed = 0.0;
      current_position[i].acceleration = 0.0;
      std::cout << "current_position["<<i<<"] = " << current_position[i].position*180/M_PI  << std::endl;
  }
  current_position.time =  base::Time::fromSeconds(0);

  std::cout << "Testing STOP MOTION" << std::endl;
  msgs::MotionReply reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
  std::cout << "START_TRAJ_MODE(200121) - Result: " << reply.result << " for the cmd: " << reply.command << std::endl;

  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
  std::cout << "CHECK_MOTION_READY(200101) - Result: " << reply.result << " for the cmd: " << reply.command << std::endl;

  reply = driver_ctrl.sendJointTrajPTFullCmd(0, 0, current_position);
  std::cout << "Result traj: " << reply.result << " for the cmd: " << reply.command << std::endl;
  std::cout << "SubCode: " << reply.subcode << std::endl;

  current_position[5].position = 1.55;

  for(int sequence = 1; sequence < 30; sequence++)
  {
      current_position.time =  base::Time::fromSeconds(sequence*2);
      reply = driver_ctrl.sendJointTrajPTFullCmd(0, sequence, current_position);
      if (reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
      {
          std::cout << "result is NOT SUCCESS, it is " << reply.result << " with sub code " <<reply.subcode << std::endl;
          throw std::runtime_error("Bad joint command");
      }
      current_position[5].position *= -1;
  }

  sleep(6);
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);
  std::cout << "CHECK_QUEUE_CNT(200102) - Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
  std::cout << "STOP_MOTION(200111) - Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);
  std::cout << "CHECK_QUEUE_CNT(200102) - Result: " << reply.result << " for the cmd: " << reply.command << std::endl;

  //PARAR E CONTINUAR COM TEMPO LONGO
  //PARAR COM VELOCIDADE DIFERENTE DE ZERO

  sleep(2);

  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
  //std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;
  //reply = driver_ctrl.sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
  //std::cout << "Result: " << reply.result << " for the cmd: " << reply.command << std::endl;

}
