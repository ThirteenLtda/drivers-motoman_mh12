#include <motoman_mh12/Msgs.hpp>
#include <motoman_mh12/Driver.hpp>
#include <base/JointState.hpp>
#include <base/JointsTrajectory.hpp>
#include <motoman_mh12/TestHelpers.hpp>
#include <iostream>
#include <math.h>

using namespace motoman_mh12;

int main(int argc, char **argv)
{
    Driver driver_ctrl;
    driver_ctrl.openTCP("192.168.10.77", 50240);

    Driver driver_streaming;
    driver_streaming.openTCP("192.168.10.77", 50241);
    driver_streaming.setExtractLastPacket(true);

    std::cout << "Connected" << std::endl;
    
    msgs::ReadSingleIo single_io = driver_ctrl.queryReadSingleIO(20031);
    std::cout << "VALUE: " << single_io.value << std::endl;
    std::cout << "RESULT_CODE: " << single_io.result << std::endl;
}

