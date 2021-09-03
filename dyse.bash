#!/usr/bin/env bash

#########
# Tools
#########

setBot(){
	export ROBOT_IP=$1
}

#########
# Aliases
#########

export ROBOT_IP=10.0.0.13 # the default static IP for dyse-robots

alias setupBot="source install/setup.bash && export ROS_MASTER_URI=${ROBOT_IP}"
alias sshBot="ssh dyse@${ROBOT_IP}"