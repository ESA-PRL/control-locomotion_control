rock_library(locomotion_control
    SOURCES LocomotionControl.cpp
    HEADERS LocomotionControl.h LocomotionControlTypes.h
    DEPS_PKGCONFIG generic_rover_manoeuvre base-lib)

#rock_executable(locomotion_control_bin Main.cpp
#    DEPS locomotion_control)

