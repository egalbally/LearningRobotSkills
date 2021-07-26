# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/05-dual_proxy_impedance_haptic)
ADD_EXECUTABLE (control_05_haptic controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_05_haptic ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
