#include <gtest/gtest.h>
#include <motoman_mh12/motoman_mh12Msgs.hpp>
#include <motoman_mh12/motoman_mh12Constants.hpp>
#include <motoman_mh12/Driver.hpp>
using namespace motoman_mh12;

class MH12MsgsFixture: public::testing::Test
{
protected:
    virtual void SetUp()
    {
     
    }
    virtual void TearDown()
    {
    } 
    Driver driver;
        
};

TEST_F(MH12MsgsFixture, extractPacketTest)
{
  msgs::StatusMsg status_msg;
  status_msg.prefix.msg_type = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  memcpy(header_uint8, &status_msg.prefix.msg_type, sizeof(status_msg.prefix.msg_type));
  EXPECT_EQ(0,driver.extractPacket(header_uint8,4));
  
  int expected_size = MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE + MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE;
  //if lenght != expected_size  (from msg_type) it must jump to the next byte
  EXPECT_EQ(-1,driver.extractPacket(reinterpret_cast<uint8_t*>(&status_msg),expected_size));
  
  status_msg.prefix.length = expected_size;
  EXPECT_EQ(expected_size,driver.extractPacket(reinterpret_cast<uint8_t*>(&status_msg),expected_size));
}

TEST_F(MH12MsgsFixture, parseStatusDriverException)
{
  msgs::StatusMsg status_msg;
  status_msg.prefix.msg_type =  MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  status_msg.drives_powered = -1; 
  int expected_size = MotomanMsgTypes::MOTOMAN_PREFIX_MSG_SIZE + MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
  
  status_msg.drives_powered = 0;
  status_msg.e_stopped = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
 
  status_msg.e_stopped = 0;
  status_msg.error_code = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
  
  status_msg.error_code = 0;
  status_msg.ln_error = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
  
  status_msg.ln_error = 0;
  status_msg.ln_motion = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
 
  status_msg.ln_motion = 0;
  status_msg.mode = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
 
  status_msg.mode = 0;
  status_msg.motion_possible = -1;
  EXPECT_THROW(driver.parsePacket(reinterpret_cast<uint8_t*>(&status_msg),MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
