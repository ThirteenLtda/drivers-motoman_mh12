#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <iostream>

using namespace motoman_mh12;


int main(int argc, char **argv)
{

  Driver driver;
  driver.openTCP("192.168.10.77", 50241);
 
  while(true)
  {
     try{ 
        msgs::MotomanMsgType msg_type =  driver.read();
        msgs::MotomanStatus status = driver.getRobotStatus();
        std::cout << "driver powered: " << status.drives_powered << std::endl;
        std::cout << "e stopped: " << status.e_stopped << std::endl;
        std::cout << "error code: " << status.error_code << std::endl;
        std::cout << "ln error: " << status.ln_error << std::endl;
        std::cout << "ln motion: " << status.ln_motion << std::endl;
        std::cout << "mode: " << status.mode << std::endl;
        std::cout << "motion possible: " << status.motion_possible << std::endl;
        
        std::cout << "=========" << std::endl;
        
        msgs::MotomanJointFeedback joint_feedback = driver.getJointFeedback();
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
     }
     catch(iodrivers_base::TimeoutError e)
     {
         
     }
         
  }
  
}