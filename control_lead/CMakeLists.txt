# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LEARNING_SKILLS_BINARY_DIR}/control_lead)
ADD_EXECUTABLE (lead_control_robot lead_robot_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
ADD_EXECUTABLE (lead_control_haptic lead_haptic_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
# ADD_EXECUTABLE (lead_robot_simviz lead_simviz.cpp ${LEARNING_SKILLS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (lead_control_robot ${LEARNING_SKILLS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (lead_control_haptic ${LEARNING_SKILLS_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (lead_robot_simviz ${LEARNING_SKILLS_COMMON_LIBRARIES})

# copy control launch script to runtime directory
# FILE(COPY launch_lead_control_robot_Clyde.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro_left.urdf DESTINATION ${APP_RESOURCE_DIR})
