sudo cpufreq-set -c 2 -g "performance"
taskset -c 2 ./haptic_control_robot Clyde
