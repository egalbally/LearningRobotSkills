# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/02-dual_proxy_motion_normal_force_haptic)
ADD_EXECUTABLE (control_02_haptic controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_02_haptic ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
