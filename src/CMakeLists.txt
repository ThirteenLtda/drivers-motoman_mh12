rock_library(motoman_mh12
    SOURCES Driver.cpp
    HEADERS Driver.hpp
            motoman_mh12Constants.hpp
            motoman_mh12Msgs.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)
        
