#!/usr/bin/env bash
# This is the setup file for Rofous
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

# initialize some variables for setup
DIR=$PWD
ME=Installer
ENV_NAME=dyse-env

echo [${ME}] installing from ${DIR}

helpFunction()
{
   echo -e "\n \t-e Decides the location of the default python3 environment for this project"
   echo -e "\t-name Decides the name of the python env the default is drone-env"
   echo -e "\t-p will purge the project setup\n"
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
	echo
	echo [${ME}] Done purging!
	echo
	exit 1
}

# make sure that the script is executing from Rofous root
if [[ ${DIR} == */Dyse-Robotics ]] || [[ ${DIR} == */rofous ]]; then
	echo [${ME}] Initializing repository ;
else
	echo [${ME}] Please run this script from the top directory of this repository ; 
	helpFunction
fi

# get user inputs, null will quit
while getopts e:name:p: opt
do
	case ${opt} in
    	e) ENV_PATH=${OPTARG} ;;
		name) ENV_NAME=${OPTARG} ;;
		p) purgeFunction ;;
    	?) helpFunction ;; # Print helpFunction in case parameter is non-existent
	esac
done

# force user to provide python environment path
checkInputs

# install python3 dependencies
sudo apt-get update
python3 -m venv ${ENV_PATH}/${ENV_NAME}
source ${ENV_PATH}/${ENV_NAME}/bin/activate
echo [${ME}] Creating your default environment
pip install --upgrade pip
pip install --ignore-installed -r ${DIR}/python3_requirements.txt
deactivate

echo "alias load_tools='PYTHONPATH=$PYTHONPATH:${DIR}/dyse_tools && source ${ENV_PATH}/${ENV_NAME}/bin/activate'" >> ~/.bashrc

echo
echo [${ME}] Successful Install
echo