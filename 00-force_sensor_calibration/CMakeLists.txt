# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/00-force_sensor_calibration)
ADD_EXECUTABLE (bias_measurement bias_measurement.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (tool_calibration tool_calibration.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (bias_measurement ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (tool_calibration ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
