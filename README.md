## Summary

This repository hosts material for Homework 3, in which we use Webots to simulate a robot capable of chasing a blue ball when in autonomous mode.

## Webots simulation

A Webots project is enclosed, which provides a Pioneer 3dx robot fitted with an additional camera device.

## Webots bridge

A key element in simulation is fitting the simulator to the product under development, as they may have different conventions, technical constraints, or ergonomic ramifications.

We'd like to run the native Webots Mac application, because tunneling graphics drivers is complicated, as is dual-booting.

But installing ROS on Mac is also complicated. Instead, we'd prefer to use a prebuilt Docker image from the ROS community (ros:noetic) to run the robot in a container running Linux and ROS.

Webots provides [few tools](https://cyberbotics.com/doc/guide/using-ros) to interface Webots and ROS. The ideal scenario is when both sides are on the same Linux host, since a built-in solution is provided. In other scenarios that share a host, a single piece of code can be written, bridging the Webots and ROS APIs to serve both as a Webots [extern controller](https://cyberbotics.com/doc/guide/running-extern-robot-controllers) and a ROS node (example from docs: [ros_python.py](https://docs.ros.org/en/melodic/api/webots_ros/html/ros__python_8py_source.html)).

Our custom code uses the `extern controller` technique, but since the Webots and ROS components are on different hosts, the APIs can't be bridged through direct method calls. Instead, a TCP socket is established between a ROS node endpoint and a Webots controller endpoint, and the two sides communicate with each other by reading and writing serialized messages.

## Running the robot

### Step 1 – Install and start lima (an analogue for Docker[^docker_coexist])
1. `brew install lima`
1. `limactl start`

[^docker_coexist]: I believe Docker and Lima can coexist.

### Step 2 – Open the robot world in Webots
Double-click `sim.wbt` under `webots-simulation/worlds`[^auto_open].

[^auto_open]: If you ever want to automate this, you can use `open -a Webots webots-simulation/worlds/sim.wbt`.

### Step 3 – Start the robot
In a dedicated terminal, run `./startup/start_guest.sh robo_chaser 9999`

This starts a Docker container and runs the robot within it. To have the robot chase the ball, enter `a` to put it in auto mode -- it will start chasing once the bridge is up and running.

### Step 4 – Start the webots bridge
In another dedicated terminal, run `./startup/start_host.sh 9999`

### Step 5 – Drive the robot
Type commands into the robot, and watch it move in Webots.
