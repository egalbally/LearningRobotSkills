# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LEARNING_SKILLS_BINARY_DIR}/haptic_control)
#ADD_EXECUTABLE (controller_gui controller_gui.cpp)
#ADD_EXECUTABLE (simviz_gui simviz_gui.cpp)
ADD_EXECUTABLE (haptic_control_robot robot_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})
ADD_EXECUTABLE (haptic_control_device device_controller.cpp ${LEARNING_SKILLS_COMMON_SOURCE})

# and link the library against the executable
#TARGET_LINK_LIBRARIES (controller_gui ${CS225A_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (simviz_gui ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_control_robot ${LEARNING_SKILLS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_control_device ${LEARNING_SKILLS_COMMON_LIBRARIES})

# copy example interface to output
# FILE(COPY gui-construction.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build

#FILE(COPY ../joint/mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
#FILE(COPY ../joint/world2.urdf DESTINATION ${APP_RESOURCE_DIR})

SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_allegro_left.urdf DESTINATION ${APP_RESOURCE_DIR})

#FILE(COPY ../weld/mmp_panda_weld.urdf DESTINATION ${APP_RESOURCE_DIR})
#FILE(COPY ../weld/world_weld.urdf DESTINATION ${APP_RESOURCE_DIR})

# FILE(COPY ../model/YZ_steel DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
#FILE(COPY launch_gui.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


