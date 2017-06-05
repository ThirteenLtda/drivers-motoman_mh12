rock_library(motoman_mh12
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motoman_mh12_bin Main.cpp
    DEPS motoman_mh12)

