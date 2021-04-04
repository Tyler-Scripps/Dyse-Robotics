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
PARAMS[PROJECT_NAME]=Dyse-Robotics

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
	echo -e "\t[Option]\t Description"
	echo -e "\n\t-e\t specifies the path to the python3 environment for this project\n"
	echo -e "\t-n\t decides the name of the python env\n"
	echo -e "\t-p\t declares the name of the project, the default is Dyse-Robotics\n"
	echo -e "\t-c\t specifies the path to a new project relative to this script\n" 						# Master Only
	echo -e "\t-g\t specifies the extension of a git repo to attach to the project's root (make sure to update .gitignore before)\n"
	echo -e "\t-o\t specifies file path to load the above parameters from\n"
	echo -e "\t-b\t will build the project with catkin build\n"
	echo -e "\t-h\t prints this help message (program will exit after)\n"
	exit 1 # Exit script after printing help
}

####################
# Validating methods
####################

isEmptyInput()
{
	# Deprecated
	for val in "$@"
	do
		if [[ -z $val ]]; then
			echo -e "[$ME]\tMissing $val"
			helpFunction
		fi
	done
}

isRootConnected()
{
	# make sure that the script is executing from the projects root
	if [[ $DIR = */$1 ]]; then
		logInfo  "Confirmed $DIR is your project's root"
	else
		logInfo "Could not find your project's root directory"
		logInfo "Project Name: $1"
		logInfo "Current directory: $DIR"
		helpFunction
	fi
}

####################
# Functional methods
####################

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

aptWrap()
{
	sudo apt-get update
	$1 sudo apt-get install $2
}

initPy3Env()
{
	python3 -m venv $1
	if [[ -f python3_requirements.txt ]]; then
		source $1/bin/activate
		pip install -r python3_requirements.txt
		deactivate
	fi
}

attachRemote()
{
	if [[ ! -f *.git* ]]; then
		git init
		git add .
		git commit -m "[$ME]    Automated commit"
		git remote add origin "https://github.com/$1"
	fi

	git pull origin master
}

writeConfig()
{
	if [[ -n $3 ]]; then
		filePath=$1/$2/$3
		echo "PROJECT_NAME=$2" > $filePath
		logInfo "Writing to $filePath"
		for key in ${!PARAMS[@]}; 
		do
			echo $key=${PARAMS[$key]} >> $filePath
		done
	else
		logInfo "Invalid Config file"
		helpFunction
	fi
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

spawnProject()																							# Master Only
{																										# Master Only
	if [[ ! -d $1/$2 ]]; then																			# Master Only
		logInfo "Creating new project space"															# Master Only
		mkdir -p $1/$2 																					# Master Only
	fi 																									# Master Only
	sed /"# Master Only"/d $DIR/setup.bash >> $1/$2/setup.bash 											# Master Only
	sed -i /PARAMS[ENV_NAME]=Dyse_env/c\PARAMS[ENV_NAME]=$2_env $1/$2/setup.bash 						# Master Only
	sed -i /PROJECT_MASTER=/c\PROJECT_MASTER=$DIR $1/$2/setup.bash 										# Master Only
	chmod +x $1/$2/setup.bash 																			# Master Only
	writeConfig $1 $2 $3																				# Master Only
	cd $1/$2																							# Master Only
	logInfo "Transfering Control"																		# Master Only
	./setup.bash -o $3																					# Master Only
	cd $DIR 																							# Master Only
	exit 1																								# Master Only
}																										# Master Only

makeProjectSpace()
{
	logInfo "Working from $DIR"
	logInfo ${PARAMS[@]}
	if [[ -z ${PARAMS[DIR_EXT]} && -n ${PARAMS[CONFIG]} ]]; then
		loadConfig ${PARAMS[CONFIG]}
	fi

	if [[ -n ${PARAMS[DIR_EXT]} ]]; then																# Master Only
		logInfo "Checking isolated project space"														# Master Only
		spawnProject ${PARAMS[DIR_EXT]} ${PARAMS[PROJECT_NAME]} ${PARAMS[CONFIG]}					 	# Master Only
	fi 																									# Master Only

	logInfo "Asserting runtime location"
	isRootConnected ${PARAMS[PROJECT_NAME]}

	if [[ -n ${PARAMS[GIT_EXT]} ]]; then
		logInfo "Attaching remote git repository"
		attachRemote ${PARAMS[GIT_EXT]}
	fi

	if [[ -f dependencies.txt ]]; then
		logInfo "Installing dependencies"
		aptWrap xargs <dependencies.txt
	fi

	if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			logInfo "Creating Python3 Environment"
			initPy3Env ${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}
	fi

	if [[ ${PARAMS[DO_BUILD]} = true ]]; then
		logInfo "Building your workspace"
		if [[ ! -d src ]]; then
			mkdir src
		fi
		catkin config -init
		if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			catkin config -DPYTHON_EXECUTABLE=${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}/bin/python3
		else
			catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3
		fi
		catkin build ${PARAMS[BUILD]}
	fi

	logInfo "Project space successfully Built!"
}


################
# Process script
################

logInfo "Filling parameter table"
while getopts "e:n:p:c:g:o:b h:" opt;
do
	case $opt in
		e) PARAMS[ENV_PATH]=$OPTARG ;;
		n) PARAMS[ENV_NAME]=$OPTARG ;;
		p) PARAMS[PROJECT_NAME]=$OPTARG ;;
		c) PARAMS[DIR_EXT]=$OPTARG ;;																	# Master Only
		g) PARAMS[GIT_EXT]=$OPTARG ;;
		o) PARAMS[CONFIG]=$OPTARG ;;
		b) PARAMS[BUILD]=$OPTARG PARAMS[DO_BUILD]=true ;;
		h) helpFunction ;;
		?) helpFunction ;;
	esac
done

makeProjectSpace