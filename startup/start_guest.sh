#!/bin/bash

set -xe

die() { echo "$*" 1>&2 ; exit 1; }

STARTUP_DIR=$(cd "$(dirname "$0")" && pwd)
BASE_DIR="$STARTUP_DIR"/..

DOCKER_TAG=$1
! [[ -z "$DOCKER_TAG" ]] || die "Please specify a tag for the Docker build."

PORT=$2
! [[ -z "$PORT" ]] || die "Please specify port."

nerdctl.lima build -t "$DOCKER_TAG" "$BASE_DIR"

exec nerdctl.lima run -it --rm -p $PORT:$PORT "$DOCKER_TAG" \
	"bash" \
	"-c" \
	"
		roscore &
		python3 -u /usr/local/bin/ros_endpoint.py $PORT &
		python3 -u /robot/opencv_node.py topic &
		python3 -u /robot/drive_node.py &
		python3 -u /robot/command_node.py
	"