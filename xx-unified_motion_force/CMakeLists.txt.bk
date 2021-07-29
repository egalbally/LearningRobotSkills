# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SAI2-EXAMPLES_BINARY_DIR}/02-redundant_arm)
ADD_EXECUTABLE (controller02 controller.cpp)
ADD_EXECUTABLE (simviz02 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller02 ${SAI2-EXAMPLES_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz02 ${SAI2-EXAMPLES_COMMON_LIBRARIES})

# copy example interface to output
FILE(COPY 02-redundant_arm.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY kuka_iiwa.jpg DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})