#!/bin/bash

set -xe

# we don't have readlink -f, and we can't count on having coreutils from brew
DEMO_DIR=$(cd "$(dirname "$0")" && pwd)
BASE_DIR="$DEMO_DIR"/..

PORT=9999

TMP_SPACE=$(mktemp -d /tmp/ros_webots_demo.XXXXXXXX)
VENV="$TMP_SPACE"/pyenv
python3 -m venv "$VENV"
"$VENV"/bin/pip install --upgrade pip "$BASE_DIR"/python_webots_bridge

# not using tags here since we don't want this demo to have side effects
# --iidfile didn't work, oddly
echo "Building docker image silently, please wait"
DOCKER_IMG=$(nerdctl.lima build -q "$BASE_DIR")

# start the ROS side
nerdctl.lima run -i --rm -p $PORT:$PORT -v "$DEMO_DIR":/tmp/demo "$DOCKER_IMG" bash << EOF &
    roscore &
    python3 -u /usr/local/bin/ros_endpoint.py 9999 &
    python3 -u /tmp/demo/demo_robot.py
    exit
EOF

echo "Waiting a few seconds for ROS to start"
sleep 5

# start the Webots side
WEBOTS_HOME=/Applications/Webots.app
# Assume if we're on arm then we're not using brew
if [ $(arch) = "arm64" ]
then
    export PYTHONPATH=$PYTHONPATH:"$WEBOTS_HOME"/lib/controller/python39
else
    export PYTHONPATH=$PYTHONPATH:"$WEBOTS_HOME"/lib/controller/python39_brew
fi
env \
    DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH":"$WEBOTS_HOME"/lib/controller \
    arch -arch X86_64 "$VENV"/bin/webots_endpoint.py localhost $PORT