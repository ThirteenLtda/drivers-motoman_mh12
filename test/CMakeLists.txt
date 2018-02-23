#rock_gtest(MH12RobotStatusTest
#   suite.cpp
#   MH12RobotStatusTest.cpp
#   HEADERS
#   LIBS ${Boost_SYSTEM_LIBRARY} 
#   DEPS motoman_mh12)
   
# rock_gtest(MH12TrajPtFullTest
#    suite.cpp
#    MH12TrajPtFullTest.cpp
#    HEADERS
#    LIBS ${Boost_SYSTEM_LIBRARY} 
#    DEPS motoman_mh12)
#    
rock_gtest(test_Driver
   test_Driver.cpp
   HEADERS
   LIBS ${Boost_SYSTEM_LIBRARY} 
   DEPS motoman_mh12)

rock_executable(test_control_mh12
    test_driver_mh12.cpp
    DEPS motoman_mh12)

rock_executable(test_robot_streaming
    test_robot_streaming.cpp

rock_executable(test_stop_command
    test_stop_command.cpp
    DEPS motoman_mh12)

    DEPS motoman_mh12)