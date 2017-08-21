#include <gtest/gtest.h>
#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <iodrivers_base/FixtureGtest.hpp>
#include <iodrivers_base/Exceptions.hpp>


using namespace motoman_mh12;

struct DriverTest : public ::testing::Test, public iodrivers_base::Fixture<Driver>
{
    DriverTest()
    {
        driver.openURI("test://");
    }
};

TEST_F(DriverTest, it_fails_when_no_header_matches)
{
    uint32_t msg[] = {0, 0, 0, 0, 0};
    ASSERT_EQ(-1, driver.extractPacket(reinterpret_cast<uint8_t*>(&msg), 20) );
}

TEST_F(DriverTest, it_waits_when_the_message_is_not_complete)
{
    uint32_t msg[] = {40,13,0,0,0};
    ASSERT_EQ(0, driver.extractPacket(reinterpret_cast<uint8_t*>(&msg), 20) );
}

TEST_F(DriverTest, it_waits_until_a_valid_message_arrives)
{
    uint32_t msg[] = {40,13,0,0,0};
    uint8_t *buffer = reinterpret_cast<uint8_t*>(&msg);
    pushDataToDriver(buffer, buffer + 20); 
    uint32_t msg2[] = {0, 0, 0, 0, 0, 0};
    buffer = reinterpret_cast<uint8_t*>(&msg);
    pushDataToDriver(buffer, buffer + 24); 
    ASSERT_EQ(msgs::MOTOMAN_ROBOT_STATUS, driver.read() );
}

TEST_F(DriverTest, it_reads_correctly_the_possible_messages_headers)
{
    uint32_t status_msg[] = {40, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t *buffer = reinterpret_cast<uint8_t*>(&status_msg);
    pushDataToDriver(buffer, buffer + 44);
    ASSERT_EQ(msgs::MOTOMAN_ROBOT_STATUS, driver.read() );
    
    msgs::JointFeedbackMsg joint_feedback;
    joint_feedback.prefix.length = 36*4;
    joint_feedback.prefix.msg_type = msgs::MOTOMAN_JOINT_FEEDBACK;
    joint_feedback.robot_id = 0;
    joint_feedback.valid_field = 2;
    joint_feedback.time = 0.0;
    for(int i=0; i<10; i++)
    {
        joint_feedback.positions[i] = 0.0;
        joint_feedback.velocities[i] = 0.0;
        joint_feedback.accelerations[i] = 0.0;
    }
    buffer = reinterpret_cast<uint8_t*>(&joint_feedback);
    pushDataToDriver(buffer, buffer + joint_feedback.prefix.length);
    ASSERT_EQ(msgs::MOTOMAN_JOINT_FEEDBACK, driver.read() );
    
    uint8_t jib[] = {0,0,0,0,0};
    buffer = reinterpret_cast<uint8_t*>(&jib);
    pushDataToDriver(buffer, buffer + 20);
    ASSERT_THROW(driver.read(), iodrivers_base::TimeoutError);
    
}

TEST_F(DriverTest, sendMotionCtrl)
{
    IODRIVERS_BASE_MOCK();
    msgs::MotionCtrlMsg expectation(0,0,msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
    uint8_t* exp = reinterpret_cast<uint8_t*>(&expectation);
    std::vector<uint8_t> expv(exp, exp +msgs::MOTOMAN_MOTION_CTRL_SIZE);
    
    msgs::MotionReplyMsg reply(0,0,msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY,0, 0);
    uint8_t* rep = reinterpret_cast<uint8_t*>(&reply);
    std::vector<uint8_t>repv(rep, rep + msgs::MOTOMAN_MOTION_REPLY_SIZE);
   
    EXPECT_REPLY(expv, repv);
    driver.sendMotionCtrl(0,0,msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
}

TEST_F(DriverTest, sendJointTrajPTFullCmd)
{
    uint8_t expected_command[] = {
	0x94, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xa0, 0x40, 0xc3, 0xf5, 0xc8, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    uint8_t expected_reply[] = {
	0x48, 0x00, 0x00, 0x00, 0xd2, 0x07, 0x00, 0x00, 0x03, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00
    };

    IODRIVERS_BASE_MOCK();

    std::vector<base::JointState> current_position;
    current_position.push_back(base::JointState::Position(1.57));
    for(int i=0; i<9; i++)
    {
        current_position.push_back(base::JointState::Position(0));
    }
    EXPECT_REPLY(
	std::vector<uint8_t>(expected_command, expected_command + sizeof(expected_command)),
	std::vector<uint8_t>(expected_reply, expected_reply + sizeof(expected_reply)));
    msgs::MotionReply reply = driver.sendJointTrajPTFullCmd(0, 1, base::Time::fromSeconds(5), current_position);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
