
#ifndef MOTOMAN_MH12_DRIVER_HPP
#define MOTOMAN_MH12_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include "motoman_mh12Msgs.hpp"

namespace motoman_mh12
{
    class Driver : public iodrivers_base::Driver
    {
        std::vector<uint8_t> buffer;
        
    public:
        Driver();
        /** Read available packets on the I/O */
        int read();
        
        int extractPacket (uint8_t const *buffer, size_t buffer_size) const;
        
        int returnMsgSize(int msg_type) const;
        int parsePacket(uint8_t const* buffer, size_t size);
        msgs::MotomanStatus parseReadStatus(uint8_t const* buffer, size_t size) const;
        msgs::MotomanJointFeedback parseJointFeedback(uint8_t const* buffer) const;
        void parseMotionReply(uint8_t const* buffer);
        bool waitForReply(base::Time const& timeout, int32_t msg_type);
        void sendJointTrajPTFullCmd(int robot_id, int sequence, base::Time timestamp,
                                    std::vector<base::JointState> joint_states);
        void readJointFeedback(base::Time const& timeout);
        
        void sendMotionCtrl(int robot_id, int sequence, int cmd);
        void readMotionCtrlReply(base::Time const& timeout);
        
        void sendReadSingleIO(int IOaddress);
        void readSingleIOReply(const base::Time& timeout);
        void parseReadSingleIOReply(uint8_t const* buffer);

        
        void sendWriteSingleIo(int IOaddress, int value);
        void readWriteSingleIO(const base::Time& timeout);
        void parseWriteSingleIOReply(uint8_t const* buffer);

        
        msgs::MotomanStatus status;
        msgs::MotomanJointFeedback joint_feedback;
    };
}

#endif
