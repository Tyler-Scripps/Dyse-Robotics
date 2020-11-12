#!/usr/bin/env bash
# This is the setup file for Dyse-Tools
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

# initialize some variables for setup
DIR=$PWD
ME=Installer
ENV_NAME=dyse-env
FROM_SAVE=false
PROJECT_NAME=null


echo [${ME}] installing from ${DIR}

helpFunction()
{
   echo -e "\n \t-e Decides the location of the default python3 environment for this project"
   echo -e "\t-name Decides the name of the python env the default is drone-env"
   echo -e "\t-p will purge the project setup"
   echo -e "\t-project creates a new project in the projects folder\n"
   echo -e "\t-git will load a project from git"
   exit 1 # Exit script after printing help
}

checkInputs()
{
	if [ -z ${ENV_PATH} ]; then
		echo -e [${ME}] Missing default python3 environment path
		helpFunction
	fi
}

purgeFunction()
{
	checkInputs
	echo
	echo [${ME}] purging the setup ...
	echo
	if [[ ${PTHONPATH} == */${ENV_PATH}/* ]]; then
		deactivate
	fi
	PYTHONPATH=
	echo -e [${ME}] removing:" \n\t $ENV_PATH/$ENV_NAME"
	sudo rm -rf ${ENV_PATH}/${ENV_NAME}
	sed -i '/dyse/d' ~/.bashrc
	echo
	echo [${ME}] Done purging!
	echo
	exit 1
}

# make sure that the script is executing from Rofous root
if [[ ${DIR} == */Dyse-Robotics ]]; then
	echo [${ME}] Initializing repository ;
	git checkout mdsdev
else
	echo [${ME}] Please run this script from the top directory of this repository ; 
	helpFunction
fi

# get user inputs, null will quit
while getopts e:name:p:project opt
do
	case ${opt} in
    	e) ENV_PATH=${OPTARG} ;;
		name) ENV_NAME=${OPTARG} ;;
		p) purgeFunction ;;
		project) PROJECT_NAME=${OPTARG} ;;
		git) PROJECT_NAME=${OPTARG} FROM_SAVE=true;;
    	?) helpFunction ;; # Print helpFunction in case parameter is non-existent
	esac
done

# force user to provide python environment path
checkInputs

if [[ ${PROJECT_NAME} == null ]]; then

	# install python3 dependencies
	sudo apt-get update 
	python3 -m venv ${ENV_PATH}/${ENV_NAME} 
	source ${ENV_PATH}/${ENV_NAME}/bin/activate 
	echo [${ME}] Creating your default environment 
	pip install --upgrade pip 
	pip install --ignore-installed -r ${DIR}/python3_requirements.txt 
	deactivate

	echo "alias load_tools='PYTHONPATH=$PYTHONPATH:${DIR}/dyse_tools && source ${ENV_PATH}/${ENV_NAME}/bin/activate'" >> ~/.bashrc

else if [[ ${FROM_SAVE} == true ]]; then
	load_tools
	echo [${ME}] cloning ${PROJECT_NAME}
	cd Projects
	git clone https://github.com/mithellscott/${PROJECT_NAME}
	cd ${PROJECT_NAME}
	pip install --ignore-installed -r python3_requirements.txt
	catkin_make
	deactivate

else
	load_tools
	echo [${ME}] creating ${PROJECT_NAME}
	cd Projects
	mkdir ${PROJECT_NAME}
	cd ${PROJECT_NAME}
	mkdir Resources
	catkin_make -DPYTHON_EXECUTABLE=/usr/bn/python3
	git init
	git add .
	git commit -m "init commmit"
	git remote add origin git@github.com:mithellscott/${PROJECT_NAME}.git
	git push -u origin master
fi


echo
echo [${ME}] Setup Successful
echo