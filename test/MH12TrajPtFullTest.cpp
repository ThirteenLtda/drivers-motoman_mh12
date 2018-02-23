#include <gtest/gtest.h>
#include <motoman_mh12/Msgs.hpp>
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
  int32_t status_header = msgs::MOTOMAN_JOINT_FEEDBACK;
  uint8_t header_uint8[msgs::MOTOMAN_JOINT_FEEDBACK_SIZE-1];
  memcpy(header_uint8, &status_header, sizeof(status_header));
  EXPECT_EQ(0,driver.extractPacket(header_uint8, (msgs::MOTOMAN_JOINT_FEEDBACK_SIZE - 1) ));
  
  int32_t full_msg[msgs::MOTOMAN_JOINT_FEEDBACK_SIZE/4] = {msgs::MOTOMAN_JOINT_FEEDBACK,0,0,1};
  uint8_t full_msg_uint8[msgs::MOTOMAN_ROBOT_STATUS_SIZE -1];
  memcpy(full_msg_uint8, &full_msg, sizeof(full_msg));
//  EXPECT_EQ(0, driver.extractPacket(full_msg_uint8, msgs::MOTOMAN_JOINT_TRAJ_PT_FULL_TIME_SIZE - 1));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
