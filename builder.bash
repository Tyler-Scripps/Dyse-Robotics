#!/usr/bin/env bash
# This is the setup file for Dyse-Tools
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

############
# Parameters
############

DIR=$PWD
ME='SETUP BOI'
PROJECT_MASTER=

declare -gA PARAMS

##############################
# Assistive/Diagnostic methods
##############################

logInfo()
{
	for val in "$@"
	do
		echo -e "[$ME]\t${val}"
	done
}

helpFunction()
{
	echo -e "[$ME]\tUsage and Hints:\n"
	echo -e "      [Option]\t Description"
	echo -e "\n\t-e\t specifies the path to the python3 environment for this project"
	echo -e "\t-d\t decides the name of the python env"
	echo -e "\t-n\t specifies the project name"
	echo -e "\t-c\t specifies the config file to load from"
	echo -e "\t-b\t will build the project with catkin build (builds whatever package you pass)"
	echo -e "\t-p\t declares the name of the project (required)"
	echo -e "\t-i\t tells the program where to install the bin"
	echo -e "\t-h\t prints this help message (program will exit after)"
	exit 1 # Exit script after printing help
}																										

buildWorkSpace()
{
	logInfo "Working from $DIR"

	if [[ -f ${PARAMS[CONFIG]} ]]; then
		logInfo "Loading Configuration ${PARAMS[CONFIG]}"
		loadConfig ${PARAMS[CONFIG]}
	fi

	logInfo ${PARAMS[@]}

	if [[ -z ${PARAMS[PROJECT_NAME]} ]]; then
		logInfo "Found Empty Parameter Table"
		helpFunction
	fi																							

	if [[ -n PARAMS[DEPENDENCIES] && -f ${DIR}/config/dependencies.txt ]]; then
		logInfo "Installing dependencies from ${DIR}/config/dependencies.txt"
		aptWrap xargs <${DIR}/config/dependencies.txt -y
	fi

	if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			logInfo "Creating Python3 Environment"
			initPy3Env ${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}
			logInfo "Done creating Environment"
	fi

	if [[ -n PARAMS[DEPENDENCIES] && -f config/python3_requirements.txt ]]; then
		logInfo "Installing Python3 dependencies"
		yes | python3 -m pip install -r config/${PARAMS[PROJECT_NAME]}_python3_requirements.txt
	fi

	if [[ -n ${PARAMS[BUILD]} ]]; then
		
		logInfo "Asserting runtime location"
		isRootConnected src/dyse-robots/${PARAMS[PROJECT_NAME]}

		logInfo "Building your workspace"
		if [[ ! -d src ]]; then
			mkdir src
		fi
		catkin clean -y
		catkin config --install --profile ${PARAMS[PROFILE]}
		if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			catkin config --profile ${PARAMS[PROFILE]} -DPYTHON_EXECUTABLE=${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}/bin/python3
		else
			catkin config --profile ${PARAMS[PROFILE]} -DPYTHON_EXECUTABLE=/usr/bin/python3
		fi
		
		sudo catkin build --profile ${PARAMS[PROFILE]} ${PARAMS[BUILD]}
	fi

	if [[ -n ${PARAMS[INSTALL]} ]]; then
		logInfo "Attempting to install ${PARAMS[PROJECT_NAME]} to dyse@${PARAMS[INSTALL]}"
		# 
		
		sudo catkin build --profile ${PARAMS[PROFILE]} ${PARAMS[PROJECT_NAME]}

		sudo cp -r config /home/dyse/dyse-robotics
		sudo cp dyse.bash /home/dyse/dyse-robotics
		scp -r /home/dyse/dyse-robotics dyse@${PARAMS[INSTALL]}:/home/dyse
		sudo rm -rf /home/dyse/dyse-robotics/*

	fi
	logInfo "Project space successfully Built!"
}

################
# Process script
################
source /opt/ros/noetic/setup.bash
logInfo "Filling parameter table"
PARAMS[CONFIG]=""
while getopts "e:d:n:c:b:p:i:h:" opt;
do
	case $opt in
		e) PARAMS[ENV_PATH]=$OPTARG ;;
		d) PARAMS[ENV_NAME]=$OPTARG ;;
		n) PARAMS[PROJECT_NAME]=$OPTARG ;;
		c) PARAMS[CONFIG]=config/$OPTARG.yaml ;;
		b) PARAMS[BUILD]=$OPTARG ;;
		p) PARAMS[PROFILE]=$OPTARG ;;
		i) PARAMS[INSTALL]=$OPTARG ;;
		h) helpFunction ;;
		?) helpFunction ;;
	esac
done

source dyse.bash
buildWorkSpace