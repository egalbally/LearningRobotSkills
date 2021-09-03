# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/haptic_control)
ADD_EXECUTABLE (haptic_control_robot robot_controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (haptic_control_device device_controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})
#ADD_EXECUTABLE (robot_simviz simviz.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (haptic_control_robot ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_control_device ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (robot_simviz ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro_left.urdf DESTINATION ${APP_RESOURCE_DIR})
