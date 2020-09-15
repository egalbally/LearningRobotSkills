# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/01-RemotePandaControl_haptic)
ADD_EXECUTABLE (control_01_haptic controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_01_haptic ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
