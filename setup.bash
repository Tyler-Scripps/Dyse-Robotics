#!/usr/bin/env bash
# This is the setup file for Dyse-Tools
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

# initialize some variables for setup
DIR=$PWD
ME=Installer
ENV_NAME=Dyse-env
FROM_SAVE=false
PROJECT_NAME=null


echo [${ME}] installing from ${DIR}

helpFunction()
{
   echo -e "\n \t-e Decides the location of the default python3 environment for this project"
   echo -e "\t-n Decides the name of the python env the default is Dyse-env"
   echo -e "\t-p will purge the project setup"
   echo -e "\t-c creates a new project in the Projects folder"
   echo -e "\t-g will load a project from git\n"
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
	echo [${ME}] Initializing repository
else
	echo [${ME}] Please run this script from the top directory of this repository
	helpFunction
fi

# get user inputs, null will quit
while getopts e:n:p:c:g: opt
do
	case ${opt} in
    	e) ENV_PATH=${OPTARG} ;;
		n) ENV_NAME=${OPTARG} ;;
		p) purgeFunction ;;
		c) PROJECT_NAME=${OPTARG} ;;
		g) PROJECT_NAME=${OPTARG} FROM_SAVE=true;;
    	?) helpFunction ;; # Print helpFunction in case parameter is non-existent
	esac
done

# force user to provide python environment path
checkInputs

if [ $PROJECT_NAME == null ]; then
	# install python3 dependencies
	sudo apt-get update
	sudo apt-get install venv-python3
	python3 -m venv ${ENV_PATH}/${ENV_NAME}
	source ${ENV_PATH}/${ENV_NAME}/bin/activate
	echo [${ME}] Creating your default environment: ...
	pip install --upgrade pip
	pip install --ignore-installed -r ${DIR}/python3_requirements.txt
	deactivate
	export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages:${DIR}/dyse_tools
	echo "alias load_tools='PYTHONPATH=$PYTHONPATH && source ${ENV_PATH}/${ENV_NAME}/bin/activate'" >> ~/.bashrc

elif [ ${FROM_SAVE} == true ]; then
	echo [${ME}] cloning ${PROJECT_NAME}: ...
	cd Projects
	git clone https://github.com/mithellscott/${PROJECT_NAME}
	cd ${PROJECT_NAME}
	./setup.bash -e $ENV_PATH -n ${PROJECT_NAME}_env

else
	echo [${ME}] creating ${PROJECT_NAME}: ...
	cd Projects
	mkdir ${PROJECT_NAME}
	cd ${PROJECT_NAME}
	mkdir Python3_Notebooks
	cp ../../setup_template.bash setup.bash
	sed -i /PROJECT_NAME=Sample/c\PROJECT_NAME=${PROJECT_NAME}
	chmod +x setup.bash

	echo [${ME}] configuring catkin: ...
	catkin config -init
	catkin config -DPYTHON_EXECUTABLE=${ENV_PATH}/${ENV_NAME}/bin/python3
	catkin build


	git init
	git add .
	git commit -m "init commmit"
	git remote add origin git@github.com:mithellscott/${PROJECT_NAME}.git
	git push -u origin master
fi

git add .
git commit -m "[$ME] Auto-Commit"

echo " "
echo [${ME}] Setup Successful
echo " "
