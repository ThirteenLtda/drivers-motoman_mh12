
#ifndef MOTOMAN_MH12_DRIVER_HPP
#define MOTOMAN_MH12_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>

namespace motoman_mh12
{
    class Driver : public iodrivers_base::Driver
    {
        std::vector<uint8_t> buffer;
        
    public:
        Driver();
        
        void open(std::string const& uri);
        
        /** Read available packets on the I/O */
        void read();
        
        void close();
        int extractPacket (uint8_t const *buffer, size_t buffer_size) const;
        
        int returnMsgSize(int msg_type) const;
        void parsePacket(uint8_t const* buffer, size_t size);
        void parseReadStatus(uint8_t const* buffer, size_t size);
        void parseJointFeedback(uint8_t const* buffer, size_t size);
        void parseMotionReply(uint8_t const* buffer, size_t size);
        void parseReadSingleIOReply(uint8_t const* buffer, size_t size);
    };
}

#endif
