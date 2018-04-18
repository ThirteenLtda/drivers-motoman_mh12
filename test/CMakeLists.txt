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
    DEPS motoman_mh12)

rock_executable(test_stop_command
    test_stop_traj.cpp
    DEPS motoman_mh12)

rock_executable(test_add_to_queue
    test_add_to_queue.cpp
    DEPS motoman_mh12)

rock_executable(test_single_io
    test_single_io.cpp
    DEPS motoman_mh12)