# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/03-dual_proxy_surface_alignment_haptic)
ADD_EXECUTABLE (control_03_haptic controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_03_haptic ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
