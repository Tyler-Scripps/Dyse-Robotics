#!/usr/bin/env bash

if [[ ${PWD} != *Dyse-Robotics ]]; then
	echo -e "Please run this script from dyse-robotics root"
fi

export DYSE_ROOT=${PWD}

export FW_PORT='/dev/ttyACM0'
export ROBOT_IP=10.0.0.9 # the default static IP for dyse-robots (Rufous)
export ROBOT_ROOT=/home/dyse/dyse-robotics
export ROBOT_ADDRESS=dyse@${ROBOT_IP}

export PATH=${DYSE_ROOT}/src/dysepy:${PATH}
export PYTHONPATH=${DYSE_ROOT}/src/dysepy/bin:${PYTHONPATH}


#########
# Aliases
#########

alias Rufous="catkin clean -y && catkin build rufous && roslaunch rufous main.launch"

alias pushKey="ssh-copy-id -i ~/.ssh/id_rsa.pub dyse@${ROBOT_IP}"

alias setupBot="export ROS_IP=10.0.0.13 && export ROS_MASTER_URI=http://${ROBOT_IP} && source install/setup.bash || source devel/setup.bash"

#########
# Tools
#########

aptWrap()
{
	sudo apt-get update
	$1 sudo apt-get install $2
}

setBot(){
	export ROBOT_IP=$1
	export ROBOT_ADDRESS=dyse@$1
}

setFWPort()
{
	export FW_PORT=$1
}

flashBoard()
{
	arduino-cli compile --fqbn $1 ${DYSE_ROOT}/src/Arduino/$2 && arduino-cli upload -p ${FW_PORT} --fqbn $1 ${DYSE_ROOT}/src/Arduino/$2
}

flashNano-IOT-33()
{
	flashBoard arduino:samd:nano_33_iot $1
}

####################
# Validating methods
####################

isRootConnected()
{
	# make sure that the script is executing from the projects root
	if [[ -d $DIR/$1 ]]; then
		logInfo  "Confirmed $DIR is your project's root"
	else
		logInfo "Could not find your project's root directory"
		logInfo "Project Path: $1"
		logInfo "Current directory: $DIR"
		helpFunction
	fi
}

####################
# Functional methods
####################

installDepends(){
	aptWrap xargs <${ROBOT_ROOT}/config/dependencies.txt -y
	yes | python3 -m pip install -r ${ROBOT_ROOT}/config/$1_python3_requirements.txt
}

purgeFunction()
{
	for val in "$@"
	do
		if [[ -d $val ]]; then
			echo -e "\n[$ME]\tPurging your workspace of $val ...\n"
			sudo rm -rf $val
		else
			echo -e "\n[$ME]\t$val does not exist, skipping ...\n"
		fi	
	done
	
	echo "\n[$ME]\tDone purging!"
}

initPy3Env()
{
	python3 -m venv $1
	source $1/bin/activate
}

loadConfig()
{
	if [[ ! -f $1 ]]; then
		logInfo "There does not appear to be a config file"
		helpFunction
	else
		while IFS= read -r line; do
			PARAMS[${line%%=*}]=${line#*=}
		done < $1
	fi
}



