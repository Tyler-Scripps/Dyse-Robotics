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
	echo -e "\t-n\t decides the name of the python env"
	echo -e "\t-p\t declares the name of the project (required)"
	echo -e "\t-o\t specifies file path to load the above parameters from"
	echo -e "\t-b\t will build the project with catkin build (builds whatever package you pass)"
	echo -e "\t-h\t prints this help message (program will exit after)"
	exit 1 # Exit script after printing help
}

####################
# Validating methods
####################

paramsEmpty_eh()
{
	# Deprecated
	safe=false
	for val in $@
	do
		logInfo $val
		if [[ -z ${PARAMS[$val]} ]]; then
			logInfo "Missing $val"
		else
			safe=true
		fi
	done
	return $safe
}

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
	source $1/bin/activate
}

attachRemote()
{
	# DEPRECATED

	if [[ ! -f *.git* ]]; then
		git init
		git remote add origin "git@github.com:$1"
	fi

	git pull origin master
}

writeConfig()
{
	filePath=$1/$2/config
	if [[ ! -d $filePath ]]; then
		mkdir -p $filePath
	fi
	echo "PROJECT_NAME=$2" > $filePath/$3
	logInfo "Writing to $filePath"
	for key in ${!PARAMS[@]}; 
	do
		echo $key=${PARAMS[$key]} >> $filePath/$3
	done

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

spawnProject()																							
{				
	# DEPRECATED 																						
	if [[ ! -d $1/$2 ]]; then																			
		logInfo "Creating new project space"															
		mkdir -p $1/$2 																					
	fi 																									
	if [[ -f $1/$2/setup.bash ]]; then 																	
		rm $1/$2/setup.bash																				
	fi 																									
	# sed /""/d $DIR/setup.bash >> $1/$2/setup.bash 											
	# sed -i /PROJECT_MASTER=/c\PROJECT_MASTER=$DIR $1/$2/setup.bash 										
	# chmod +x $1/$2/setup.bash 																			
	# writeConfig $1 $2 ${PARAMS[PROJECT_NAME]}.yaml																
	# cd $1/$2																							
	# logInfo "Transfering Control"																		

	# ./setup.bash -o ${PARAMS[PROJECT_NAME]}															

	cd ${DIR}/${PARAMS[DIR_EXT]}/${PARAMS[PROJECT_NAME]}																							
	# exit 1																								
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

	logInfo "Asserting runtime location"
	isRootConnected src/dyse-robots/${PARAMS[PROJECT_NAME]}

	if [[ -z PARAMS[DEPENDENCIES] && -f ${DIR}/config/dependencies.txt ]]; then
		logInfo "Installing dependencies from ${DIR}/config/dependencies.txt"
		aptWrap xargs <${DIR}/config/dependencies.txt -y
	fi

	if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			logInfo "Creating Python3 Environment"
			initPy3Env ${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}
			logInfo "Done creating Environment"
	fi

	if [[ -z PARAMS[DEPENDENCIES] && -f config/python3_requirements.txt ]]; then
		logInfo "Installing Python3 dependencies"
		yes | python3 -m pip install -r config/${PARAMS[PROJECT_NAME]}_python3_requirements.txt
	fi

	if [[ ${PARAMS[DO_BUILD]} = true ]]; then
		logInfo "Building your workspace"
		if [[ ! -d src ]]; then
			mkdir src
		fi

		catkin config --install --profile dev
		if [[ -n ${PARAMS[ENV_PATH]} && -n ${PARAMS[ENV_NAME]} ]]; then
			catkin config -DPYTHON_EXECUTABLE=${PARAMS[ENV_PATH]}/${PARAMS[ENV_NAME]}/bin/python3
		else
			catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3
		fi
		if [[ -n ${PARAMS[BUILD]} ]]; then
			catkin build ${PARAMS[BUILD]}
		else
			catkin build ${PARAMS[PROJECT_NAME]}
		fi
	fi

	if [[ -n ${PARAMS[INSTALL]} ]]; then
		logInfo "Attempting to install ${PROJECT_NAME} to ${PARAMS[INSTALL]}"
		# 
		git add .
		git commit -m "${ME} auto commit"
		git push -f origin install

		CMD="rm -rf dyse-robotics || git clone https://github.com/mitchelldscott/dyse-robotics.git && ll && cd dyse-robotics && git checkout install && ./builder.bash -o ${PARAMS[CONFIG]}"

		ssh dyse@${PARAMS[INSTALL]} $CMD

	fi
	logInfo "Project space successfully Built!"
}

################
# Process script
################

logInfo "Filling parameter table"
PARAMS[CONFIG]=""
while getopts "e:n:p:c:o:b h:" opt;
do
	case $opt in
		e) PARAMS[ENV_PATH]=$OPTARG ;;
		n) PARAMS[ENV_NAME]=$OPTARG ;;
		p) PARAMS[PROJECT_NAME]=$OPTARG ;;
		c) PARAMS[DIR_EXT]=$OPTARG ;;																	
		o) PARAMS[CONFIG]=config/$OPTARG.yaml ;;
		b) PARAMS[BUILD]=$OPTARG PARAMS[DO_BUILD]=true ;;
		i) PARAMS[INSTALL]=$OPTARG ;;
		h) helpFunction ;;
		?) helpFunction ;;
	esac
done

source dyse.bash
buildWorkSpace