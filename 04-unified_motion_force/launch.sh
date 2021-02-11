#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./simviz02 &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID  
}

sleep 2 

# launch controller
./controller02 &
CONTROLLER_PID=$!

sleep 1

# launch interfaces server
python3 interface/server.py 02-redundant_arm.html &
SERVER_PID=$!

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller & interfaces server
kill $CONTROLLER_PID
for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
