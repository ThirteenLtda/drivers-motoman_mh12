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
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  memcpy(header_uint8, &status_header, sizeof(status_header));
  EXPECT_EQ(0,driver.extractPacket(header_uint8,10));
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,0,0,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_EQ(35,driver.extractPacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE));

  int32_t full_msg_error[8] = {0,0,0,0,0,0,0,0};
  uint8_t full_msg_error_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_error_uint8, &full_msg_error, sizeof(full_msg_error));
  EXPECT_EQ(-1,driver.extractPacket(full_msg_error_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE));
}

TEST_F(MH12MsgsFixture, parseStatusDriverException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,-1,0,0,0,0,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusEstoprException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,-1,0,0,0,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusErrorCodeException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,-1,0,0,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusLnErrorException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,0,-1,0,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusLnMotionException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,0,0,-1,0,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusModeException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,0,0,0,-1,0};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

TEST_F(MH12MsgsFixture, parseStatusMotionPossibleException)
{
  int32_t status_header = MotomanMsgTypes::MOTOMAN_ROBOT_STATUS;
  uint8_t header_uint8[4];
  
  int32_t full_msg[8] = {MotomanMsgTypes::MOTOMAN_ROBOT_STATUS,0,0,0,0,0,0,-1};
  uint8_t full_msg_uint8[MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
  EXPECT_THROW(driver.parsePacket(full_msg_uint8,MotomanMsgTypes::MOTOMAN_ROBOT_STATUS_SIZE), std::runtime_error);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
