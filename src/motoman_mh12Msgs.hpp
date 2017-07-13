#ifndef MOTOMAN_MH12_MESSAGES_HPP
#define MOTOMAN_MH12_MESSAGES_HPP

#include <stdint.h>
#include <vector>
#include <base/Time.hpp>
#include <base/JointState.hpp>

namespace motoman_mh12
{
  /*
  * \brief List of all the encapsulation of the messages received or sent to the controller DX200
  * and also the encapsulation to expose the messages to the system. Struct that end with "Msg" are
  * the ones sent/received to/by the controller
  */
  namespace msgs
  {
    //every message must start with this prefix 
    struct Prefix
    {
        int32_t length;
        int32_t msg_type;
        int32_t comm_type; //!< still unclear what is supposed to be filled here
        int32_t reply_code; //!< still unclear what is supposed to be filled here
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

    /*
    * \brief This message is used to send motion commands to control and manage the overall motion    
    */
    struct MotionCtrlMsg
    {
        Prefix prefix;
        int32_t robot_id;
        int32_t sequence;
        int32_t cmd; //!< motion command motion_ctrl::MotionControlCmd
        float data[10];
    }__attribute__((packed));
    
    /*
    * \brief MotoROS sends this reply message each time it receives 
    * a joint trajectory message or a motoman motion control command.
    */
    struct MotionReplyMsg
    {
        Prefix prefix;
        int32_t robot_id; 
        int32_t sequence; //!<Reference to the sequence number that is being responded to. 
        int32_t command; //!< Reference to the command or message type 
                         //that is being responded to. MotomanMsgType::JOINT_TRAJ_FULL 
                         //or motion_ctrl::MotionControlCmd   
        int32_t result;  //result code (motion_ctrl::MotionReplyResult)
    }__attribute__((packed));
    
    struct MotionReply
    {
        int robot_id;
        int sequence;
        int command;
        int result;
    };
    
    struct ReadSingleIoMsg
    {
        Prefix prefix;
        int32_t address; //!< Address of the controller I/O signal to be read. Values from 00010 to 1000559, please refer to the controller Concurrent I/O Manual for details on addresses.  
    }__attribute__((packed));
    
    struct ReadSingleIoReplyMsg
    {
        Prefix prefix;
        int32_t value; //State of the I/O. 0=OFF, 1=ON
        int32_t result_code; //High level command resul code: 1=SUCCESS 2= FAILURE
    }__attribute__((packed));
    
    
    struct WriteSingleIoMsg
    {
     Prefix prefix;
     int32_t io_address;  //!< Address of the controller I/O signal to be read. Values from 00010 to 1000559, please refer to the controller Concurrent I/O Manual for details on addresses.  
     int32_t value; //!< State of the I/O; 0=OFF and 1=ON.
    }__attribute__((packed));
    
    struct WriteSingleIoReplyMsg
    {
        Prefix prefix;
        int32_t result_code; //!< Result code write_single_io::ResultCode: SUCCESS = 1, FAILURE = 2.
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
    /*
    * \brief The message called JOINT_TRAJ_PT_FULL type which includes position,
    * velocity, acceleration and time is designated as type 14. This type of message 
    * contains sufficient trajectory data to accurately reproduce the trajectory 
    * generated by the compute with matching speed profile
    */
    struct JointTrajPTFullMsg
    {
        Prefix prefix;
        int32_t robot_id;
        int32_t sequence; //!< Index of point in trajectory 0 = Initial trajectory point, which should match the robot current position.
        int32_t valid_field = 7; // MotoROS expects all values, so this value should be set to 7
        float time;
        float positions[10];//!< Desired joint positions in radian. Ordering matches the sequential joint order: SLURBT for 6 axis robot and SLEURBT for 7 axis robots.
        float velocities[10]; //!< Desired joint velocities in radian/sec. Same ordering as positions.
        float accelerations[10]; //!< Desired joint accelerations in radians/sec2. Same orderins as positions.
    }__attribute__((packed));
  
  
 
  }
 
 
}
#endif    
