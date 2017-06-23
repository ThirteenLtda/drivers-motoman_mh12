
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
        void parseReadStatus(uint8_t const* buffer, size_t size);
        void parseJointFeedback(uint8_t const* buffer, size_t size);
        void parseMotionReply(uint8_t const* buffer, size_t size);
        void parseReadSingleIOReply(uint8_t const* buffer, size_t size);
        
        void sendJointTrajPTFullCmd(int robot_id, int sequence, base::Time timestamp,
                                    std::vector<base::JointState> joint_states);
        void readJointFeedback(base::Time const& timeout);
        msgs::MotomanStatus status;
        msgs::MotomanJointFeedback joint_feedback;
    };
}

#endif
