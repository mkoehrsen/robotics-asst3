#!/bin/bash

set -xe

die() { echo "$*" 1>&2 ; exit 1; }

STARTUP_DIR=$(cd "$(dirname "$0")" && pwd)
BASE_DIR="$STARTUP_DIR"/..

PORT=$1
! [[ -z "$PORT" ]] || die "Please specify port."

VENV=$(mktemp -d /tmp/python_webots_bridge.XXXXXXXX)
python3 -m venv "$VENV"
"$VENV"/bin/pip install --upgrade pip "$BASE_DIR"/python_webots_bridge

echo "Waiting a few seconds for ROS to start"
sleep 5

# start the Webots side
WEBOTS_HOME=/Applications/Webots.app
# Assume if we're on arm then we're not using brews
if [ $(arch) = "arm64" ]
then
    export PYTHONPATH=$PYTHONPATH:"$WEBOTS_HOME"/lib/controller/python39
else
    export PYTHONPATH=$PYTHONPATH:"$WEBOTS_HOME"/lib/controller/python39_brew
fi
env \
    DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH":"$WEBOTS_HOME"/lib/controller \
    arch -arch X86_64 "$VENV"/bin/webots_endpoint.py localhost $PORT