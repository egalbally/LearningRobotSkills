# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTIC_APPLICATIONS_BINARY_DIR}/02-dual_proxy_motion_normal_force_robot)
ADD_EXECUTABLE (control_02_robot controller.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_02_robot simviz.cpp ${HAPTIC_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_02_robot ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_02_robot ${HAPTIC_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro_left.urdf DESTINATION ${APP_RESOURCE_DIR})
