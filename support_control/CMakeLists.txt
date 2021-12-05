# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LEARNING_SKILLS_BINARY_DIR}/support_control)
ADD_EXECUTABLE (support_control_robot support_robot_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
ADD_EXECUTABLE (support_control_haptic support_haptic_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (support_control_robot ${LEARNING_SKILLS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (support_control_haptic ${LEARNING_SKILLS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
