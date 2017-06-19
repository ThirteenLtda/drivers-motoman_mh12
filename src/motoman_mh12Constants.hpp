#ifndef _MOTOMAN_MH12_CONSTANTS_HPP_
#define _MOTOMAN_MH12_CONSTANTS_HPP_

namespace motoman_mh12
{
  
  namespace rs232_msgs
  {
    static const uint16_t MOTOMAN_MH12_DLE = 0x10; //!<Data Link Escape
    static const uint16_t MOTOMAN_MH12_SOH = 0x01; //!<Start of Heading 
    static const uint16_t MOTOMAN_MH12_STX = 0x02; //!<Start of Text
    static const uint16_t MOTOMAN_MH12_ETX = 0x03; //!<End of Text
    static const uint16_t MOTOMAN_MH12_EOT = 0x04; //!<End of Transmission
    static const uint16_t MOTOMAN_MH12_ENQ = 0x05; //!<Enquiry
    static const uint16_t MOTOMAN_MH12_NAK = 0x15; //!<Negative Acknowledgment
    static const uint16_t MOTOMAN_MH12_ETB = 0x17; //!<End of Text Block
    static const uint16_t MOTOMAN_MH12_ACK0 = 0x30; //!<Even Affirmative Acknowledgment
    static const uint16_t MOTOMAN_MH12_ACK1 = 0x31; //!<Odd Affirmative Acknowledgment
    static const uint16_t MOTOMAN_MH12_ACK = 0x10; //!<Affirmative Acknowledgment
  }
  
  //messages used in the interface with MOTO ROS and gathered from the simple_message package
  //https://github.com/ros-industrial/motoman
  
  namespace motion_ctrl
  {
    /**
     * \brief Enumeration of motion control command codes.
     */
    namespace MotionControlCmds
    {
      enum MotionControlCmd
      {
	UNDEFINED          = 0,
	CHECK_MOTION_READY = 200101,  // check if controller is ready to receive ROS motion cmds
	CHECK_QUEUE_CNT    = 200102,  // get number of motion increments in queue
	STOP_MOTION        = 200111,  // stop robot motion immediately
	START_TRAJ_MODE    = 200121,  // prepare controller to receive ROS motion cmds
	STOP_TRAJ_MODE     = 200122,  // return motion control to INFORM
      };
    } // namespace MotionControlCmds
  }
  namespace motion_reply
  {
    
    /**
     * \brief Enumeration of motion reply result codes.
     */
    namespace MotionReplyResults
    {
      enum MotionReplyResult
      {
	SUCCESS    = 0,
	TRUE       = 0,
	BUSY       = 1,
	FAILURE    = 2,
	FALSE      = 2,
	INVALID    = 3,
	ALARM      = 4,
	NOT_READY  = 5,
	MP_FAILURE = 6
      };
    }
    
    typedef MotionReplyResults::MotionReplyResult MotionReplyResult;
    
    /*
     * \brief Enumeration of Motion reply subcodes
     */
    namespace MotionReplySubcodes
    {
      namespace Invalid
      {
	enum InvalidCode
	{
	  UNSPECIFIED = 3000,
	  MSGSIZE,
	  MSGHEADER,
	  MSGTYPE,
	  GROUPNO,
	  SEQUENCE,
	  COMMAND,
	  DATA = 3010,
	  DATA_START_POS,
	  DATA_POSITION,
	  DATA_SPEED,
	  DATA_ACCEL,
	  DATA_INSUFFICIENT
	};
      }  // namespace Invalid
      
      namespace NotReady
      {
	enum NotReadyCode
	{
	  UNSPECIFIED = 5000,
	  ALARM,
	  ERROR,
	  ESTOP,
	  NOT_PLAY,
	  NOT_REMOTE,
	  SERVO_OFF,
	  HOLD,
	  NOT_STARTED,
	  WAITING_ROS,
	  SKILLSEND
	};
      }  // namespace NotReady
    }  // MotionReplySubcodes
  }
  
  namespace io_ctrl_reply
  {
    
    /**
     * \brief Enumeration of Read Single IO reply result codes.
     */
    namespace ReadSingleIOReplyResults
    {
      enum ReadSingleIOReplyResult
      {
	FAILURE    = 0,
	SUCCESS    = 1
      };
    }
    typedef ReadSingleIOReplyResults::ReadSingleIOReplyResult ReadSingleIOReplyResult;
    
    namespace WriteSingleIOReplyResults
    {
      enum WriteSingleIOReplyResult
      {
	FAILURE    = 0,
	SUCCESS    = 1
      };
    }
    typedef WriteSingleIOReplyResults::WriteSingleIOReplyResult WriteSingleIOReplyResult;
  }
  
  namespace MotomanMsgTypes
  {
    enum MotomanMsgType
    {
      MOTOMAN_ROBOT_STATUS = 13,
      MOTOMAN_JOINT_TRAJ_PT_FULL = 14,
      MOTOMAN_JOINT_FEEDBACK = 15,
      MOTOMAN_MSG_BEGIN = 2000,
      MOTOMAN_MOTION_CTRL = 2001,
      MOTOMAN_MOTION_REPLY = 2002,
      MOTOMAN_READ_SINGLE_IO = 2003,
      MOTOMAN_READ_SINGLE_IO_REPLY = 2004,
      MOTOMAN_WRITE_SINGLE_IO = 2005,
      MOTOMAN_WRITE_SINGLE_IO_REPLY = 2006,
      ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX = 2016, // This is similar to the "Dynamic Joint Point" in REP I0001
      ROS_MSG_MOTO_JOINT_FEEDBACK_EX = 2017 //Similar to Dynamic Joint State on the REP I0001
    };
    
    enum MotomanMsgSize
    {
      MOTOMAN_ROBOT_STATUS_SIZE = 35,
      MOTOMAN_JOINT_TRAJ_PT_FULL_MIN_SIZE = 12,
      MOTOMAN_JOINT_TRAJ_PT_FULL_TIME_SIZE = 16,
      MOTOMAN_JOINT_TRAJ_PT_FULL_POS_SIZE = 52,
      MOTMAN_JOINT_TRAJ_PT_FULL_VEL_SIZE = 52,
      MOTOMAN_JOINT_TRAJ_PT_FULL_ACC_SIZE = 52,
      MOTOMAN_JOINT_TRAJ_PT_FULL_ALL_SIZE = 136
    };
    
    typedef MotomanMsgTypes::MotomanMsgType MotomanMsgType;
  }
  
 
}
#endif //_MOTOMAN_MH12_CONSTANTS_HPP_