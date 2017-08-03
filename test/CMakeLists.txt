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