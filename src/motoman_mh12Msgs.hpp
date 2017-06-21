#ifndef MOTOMAN_MH12_MESSAGES_HPP
#define MOTOMAN_MH12_MESSAGES_HPP

#include <stdint.h>
#include <boost/static_assert.hpp>
#include <vector>
#include <base/Time.hpp>
#include <base/JointState.hpp>

namespace motoman_mh12
{
  namespace msgs
  {
    struct Prefix
    {
        int32_t length;
        int32_t msg_type;
        int32_t comm_type;
        int32_t reply_code;
    }__attribute__((packed));
    
    struct StatusMsg
    {
      Prefix prefix;
      int32_t drives_powered; //!< -1=Unkown, 1=ON, 0=OFF
      int32_t e_stopped; //!< Controller E-Stop state -1 = Unknown; 1 = TRUE(ON); 0 = FALSE(OFF)
      int32_t error_code; //!< Alarm code of the first current alarm -1 = Unknown; 
      //0 = No alarm; Other = Alarm# 
      int32_t ln_error; //!< There is at least one active alarm -1 = Unknown; 
      //1 = TRUE(ON); 0 = FALSE(OFF)
      int32_t ln_motion; //!< The controller is executing a job(program) -1 = Unknown; 1 = TRUE(ON) 
      //0 = FALS(OFF)
      int32_t mode; //!< Controller/Pendant mode -1 = Unkown; 1=Manual(Teach); 0= Auto(Play or Remtote)
      int32_t motion_possible; //!< Controller can receive motion for ROS -1=Unknown; 1=Enabled; 0=Disabled
    }__attribute__((packed));
    
    struct MotionReplyMsg
    {
        Prefix prefix;
        int32_t robot_id;
        int32_t sequence;
        int32_t command;
        int32_t result;
    }__attribute__((packed));
    
    struct ReadSingleIoReplyMsg
    {
        Prefix prefix;
        int32_t value; //State of the I/O. 0=OFF, 1=ON
        int32_t result_code; //High level command resul code: 1=SUCCESS 2= FAILURE
    }__attribute__((packed));
    
    struct MotomanStatus
    {
      //encanpsulation of the message excluding the unknown states, if there is the need we can implement it later    
      bool drives_powered;
      bool e_stopped; //!< Controller E-Stop state 1 = TRUE(ON); 0 = FALSE(OFF)
      int error_code; //!< Alarm code of the first current alarm 0 = No alarm; Other = Alarm# 
      bool ln_error; //!< There is at least one active alarm 1 = TRUE(ON); 0 = FALSE(OFF)
      bool ln_motion; //!< The controller is executing a job(program) 1 = TRUE(ON) 0 = FALSE(OFF)
      bool mode; //!< Controller/Pendant mode 1=Manual(Teach); 0= Auto(Play or Remtote)
      bool motion_possible; //!< Controller can receive motion for ROS 1=Enabled; 0=Disabled
    };
    
    struct MotomanJointFeedback
    {
      int robot_id; // 0 = 1st robot
      int valid_field; // Bit-mask indicating which “optional” fields are filled with data.
		       //1 = time, 2 = position, 4 = velocity, 8 = acceleration 
		       //MotoROS only send position, so this value should be set to 2.
      base::Time time; //timestamp associated with this trajectory point in seconds
      std::vector<base::JointState> joint_states;
    };
  
  }
}
#endif    