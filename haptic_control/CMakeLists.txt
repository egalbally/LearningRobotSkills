# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LEARNING_SKILLS_BINARY_DIR}/haptic_control)
ADD_EXECUTABLE (haptic_control_robot robot_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
ADD_EXECUTABLE (haptic_control_device device_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
#ADD_EXECUTABLE (robot_simviz simviz.cpp ${LEARNING_SKILLS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (haptic_control_robot ${LEARNING_SKILLS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_control_device ${LEARNING_SKILLS_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (robot_simviz ${LEARNING_SKILLS_COMMON_LIBRARIES})

# copy control launch script to runtime directory
FILE(COPY launch_haptic_control_robot_Clyde.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro_left.urdf DESTINATION ${APP_RESOURCE_DIR})
