#ifndef MOTOMAN_MH12_MESSAGES_HPP
#define MOTOMAN_MH12_MESSAGES_HPP

#include <stdint.h>
#include <vector>
#include <base/Time.hpp>
#include <base/JointState.hpp>

namespace motoman_mh12
{
    /** \brief List of all the encapsulation of the messages received or sent to the controller DX200
     * and also the encapsulation to expose the messages to the system. Struct that end with "Msg" are
     * the ones sent/received to/by the controller
     */
    namespace msgs
    {
        enum CommType
        {
            INVALID = 0,
            TOPIC = 1,
            SERVICE_REQUEST = 2,
            SERVICE_REPLY = 3
        };
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
        static const int LENGTH_UNKNOWN = -1;
        int returnMsgSize(int msg_type);
        
        //every message must start with this prefix 
        struct Prefix
        {
            int32_t length;
            int32_t msg_type;
            int32_t comm_type; //!< still unclear what is supposed to be filled here
            int32_t reply_code; //!< still unclear what is supposed to be filled here
            
            Prefix(){}
            Prefix(msgs::MotomanMsgType msg_type):
            msg_type(msg_type),
            comm_type(0),
            reply_code(0)
            {
                length = returnMsgSize(msg_type);
            } 
            Prefix(msgs::MotomanMsgType msg_type, msgs::CommType comm_type):
            msg_type(msg_type),
            comm_type(comm_type),
            reply_code(0)
            {
                length = returnMsgSize(msg_type);
            }
            
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
            
            StatusMsg(){}
            StatusMsg(int drives_powered, int e_stopped, int error_code,
                      int ln_error, int ln_motion, int mode, int motion_possible):
                      prefix(MOTOMAN_ROBOT_STATUS),
                      drives_powered(drives_powered), 
                      e_stopped(e_stopped),
                      error_code(error_code),
                      ln_error(ln_error),
                      ln_motion(ln_motion),
                      mode(mode),
                      motion_possible(motion_possible){}
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
            float data[10] = {0};
            
            MotionCtrlMsg(int robot_id, int sequence, int cmd):
            prefix(MOTOMAN_MOTION_CTRL, SERVICE_REQUEST),
            robot_id(robot_id),
            sequence(sequence),
            cmd(cmd){}
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
            int32_t subcode;
            float data[10];
            
            MotionReplyMsg(){}
            MotionReplyMsg(int robot_id, int sequence, int command, int result, int subcode):
            prefix(MOTOMAN_MOTION_REPLY),
            robot_id(robot_id),
            sequence(sequence),
            command(command),
            result(result),
            subcode(subcode){}
        }__attribute__((packed));
        
        struct MotionReply
        {
            int robot_id;
            int sequence;
            int command;
            int result;
            int subcode;
        };
        
        struct ReadSingleIoMsg
        {
            Prefix prefix;
            int32_t address; //!< Address of the controller I/O signal to be read. Values from 00010 to 1000559, please refer to the controller Concurrent I/O Manual for details on addresses.  
            
            ReadSingleIoMsg(){}
            ReadSingleIoMsg(int addres):
            prefix(MOTOMAN_READ_SINGLE_IO),
            address(address){}
        }__attribute__((packed));
        
        struct ReadSingleIoReplyMsg
        {
            Prefix prefix;
            int32_t value; //State of the I/O. 0=OFF, 1=ON
            int32_t result_code; //High level command resul code: 1=SUCCESS 2= FAILURE
            
            ReadSingleIoReplyMsg(){}
            ReadSingleIoReplyMsg(int value, int result_code):
            prefix(MOTOMAN_READ_SINGLE_IO_REPLY),
            value(value),
            result_code(result_code){}
        }__attribute__((packed));
        
        
        struct WriteSingleIoMsg
        {
            Prefix prefix;
            int32_t io_address;  //!< Address of the controller I/O signal to be read. Values from 00010 to 1000559, please refer to the controller Concurrent I/O Manual for details on addresses.  
            int32_t value; //!< State of the I/O; 0=OFF and 1=ON.
            
            WriteSingleIoMsg(int io_address, int value):
            prefix(MOTOMAN_WRITE_SINGLE_IO),
            io_address(io_address),
            value(value){}
        }__attribute__((packed));
        
        struct WriteSingleIoReplyMsg
        {
            Prefix prefix;
            int32_t result_code; //!< Result code write_single_io::ResultCode: SUCCESS = 1, FAILURE = 2.
            
            WriteSingleIoReplyMsg(){}
            WriteSingleIoReplyMsg(int result_code):
            prefix(MOTOMAN_WRITE_SINGLE_IO_REPLY),
            result_code(result_code){}
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
        
        struct JointFeedbackMsg
        {
            Prefix prefix;
            int32_t robot_id;
            int32_t valid_field;
            float time;
            float positions[10];
            float velocities[10]; 
            float accelerations[10];
        }__attribute__((packed));
        
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
            
            JointTrajPTFullMsg():
            prefix(MOTOMAN_JOINT_TRAJ_PT_FULL, SERVICE_REQUEST){}
        }__attribute__((packed));
        
        enum MotomanMsgSize
        {
            MOTOMAN_PREFIX_MSG_SIZE = sizeof(msgs::Prefix),
            MOTOMAN_ROBOT_STATUS_SIZE = sizeof(msgs::StatusMsg) -4,
            MOTOMAN_JOINT_FEEDBACK_SIZE = sizeof(msgs::JointFeedbackMsg) -4,
            MOTOMAN_MOTION_REPLY_SIZE = sizeof(msgs::MotionReplyMsg) -4,
            MOTOMAN_READ_SINGLE_IO_REPLY_SIZE = sizeof(msgs::ReadSingleIoReplyMsg) -4,
            MOTOMAN_WRITE_SINGLE_IO_REPLY_SIZE = sizeof(msgs::WriteSingleIoReplyMsg) -4,
            MOTOMAN_JOINT_TRAJ_PT_FULL_SIZE = sizeof(msgs::JointTrajPTFullMsg) -4,
            MOTOMAN_MOTION_CTRL_SIZE = sizeof(msgs::MotionCtrlMsg) -4,
            MOTOMAN_READ_SINGLE_IO_SIZE = sizeof(msgs::ReadSingleIoMsg) -4,
            MOTOMAN_WRITE_SINGLE_IO_SIZE = sizeof(msgs::WriteSingleIoMsg) -4,
            MOTOMAN_MAX_PKT_SIZE = 136+12
        };
        /** Returns the expected packet size given a message type, if there is
         * no match, it return -1 to move the buffer pointer
         * @param msg_type Message type from the header
         */
        int returnMsgSize(int msg_type)
        {
            switch(msg_type)
            {
                case msgs::MOTOMAN_ROBOT_STATUS:
                    return msgs::MOTOMAN_ROBOT_STATUS_SIZE;
                case msgs::MOTOMAN_JOINT_FEEDBACK:
                    return msgs::MOTOMAN_JOINT_FEEDBACK_SIZE;
                case msgs::MOTOMAN_MOTION_REPLY:
                    return msgs::MOTOMAN_MOTION_REPLY_SIZE;
                case msgs::MOTOMAN_READ_SINGLE_IO_REPLY:
                    return msgs::MOTOMAN_READ_SINGLE_IO_REPLY_SIZE;
                case msgs::MOTOMAN_WRITE_SINGLE_IO_REPLY:
                    return msgs::MOTOMAN_WRITE_SINGLE_IO_REPLY_SIZE;
                case msgs::MOTOMAN_MOTION_CTRL:
                    return msgs::MOTOMAN_MOTION_CTRL_SIZE;
                case msgs::MOTOMAN_JOINT_TRAJ_PT_FULL:
                    return msgs::MOTOMAN_JOINT_TRAJ_PT_FULL_SIZE;
                case msgs::MOTOMAN_WRITE_SINGLE_IO:
                    return msgs::MOTOMAN_WRITE_SINGLE_IO_SIZE;
                default:
                    return LENGTH_UNKNOWN;
            }
        } 
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
                    BUSY       = 1,
                    FAILURE    = 2,
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
        
        namespace write_single_io
        {
            enum ResultCode
            {
                SUCCESS = 1,
                FAILURE =2
            };
        }  
        
    }
}
#endif    
