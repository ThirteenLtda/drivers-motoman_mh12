rock_library(motoman_mh12
    SOURCES Driver.cpp
    HEADERS Driver.hpp
            Msgs.hpp
            TestHelpers.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(test_limits_mh12
        test_limits_mh12.cpp
        DEPS motoman_mh12
)

rock_executable(test_src_driver_mh12
        test_src_driver_mh12.cpp
        DEPS motoman_mh12
)
