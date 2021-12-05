#!/bin/bash

# # start screen session in detached mode
# # note: this is to get around scripts running in non-interactive subshell
# sudo screen -dmS haptic_control_robot sudo nice -n -39 ./haptic_control_robot Clyde # default nice is 19 (should be 0)

# # wait until process exists
# wait $(pidof haptic_control_robot)

# # set process to RT priority
# sudo chrt -ap 99 $(pidof haptic_control_robot)

# # reattach original (interactive) screen session
# sudo screen -r haptic_control_robot

sudo nice -n -39 ./haptic_control_robot Clyde # default nice is 19 (should be 0)
