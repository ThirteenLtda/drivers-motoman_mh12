#ifndef MOTOMAN_MH12_DRIVER_HPP
#define MOTOMAN_MH12_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include "Msgs.hpp"
namespace motoman_mh12
{
class Driver : public iodrivers_base::Driver
{
    std::vector<uint8_t> buffer;

  public:
    Driver();
    /** Read available packets on the I/O */
    msgs::MotomanMsgType read(base::Time timeout);

    int extractPacket(uint8_t const *buffer, size_t buffer_size) const;

    msgs::MotomanMsgType parsePacket(uint8_t const *buffer, size_t size);
    msgs::MotomanStatus parseReadStatus(uint8_t const *buffer, size_t size) const;
    msgs::MotomanJointFeedback parseJointFeedback(uint8_t const *buffer) const;

    msgs::MotomanStatus getRobotStatus();
    msgs::MotomanJointFeedback getJointFeedback();

    void waitForReply(base::Time const &timeout, int32_t msg_type);
    msgs::MotionReply sendJointTrajPTFullCmd(int robot_id, int sequence_id, base::samples::Joints const &joints_samples);
    msgs::MotionReply sendMotionCtrl(int robot_id, int sequence_id, int cmd);
    msgs::MotionReply readMotionCtrlReply(base::Time const &timeout);
    msgs::MotionReply sendJointPosition(int sequence_id, std::vector<float> joints_positions);
    msgs::MotionReply readJointPositionReply(base::Time const &timeout);
    msgs::MotionReply parseMotionReply(uint8_t const *buffer);

    void sendReadSingleIO(int IOaddress);
    void readSingleIOReply(const base::Time &timeout);
    void parseReadSingleIOReply(uint8_t const *buffer);

    bool sendWriteSingleIo(int IOaddress, int value);
    bool parseWriteSingleIOReply(uint8_t const *buffer) const;

    msgs::MotomanStatus status;
    msgs::MotomanJointFeedback joint_feedback;
};
}

#endif
