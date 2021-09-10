# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LEARNING_SKILLS_BINARY_DIR}/force_sensor_calibration)
ADD_EXECUTABLE (bias_measurement bias_measurement.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
ADD_EXECUTABLE (tool_calibration tool_calibration.cpp ${LEARNING_SKILLS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (bias_measurement ${LEARNING_SKILLS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (tool_calibration ${LEARNING_SKILLS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
