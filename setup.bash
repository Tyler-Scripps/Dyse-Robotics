#!/bin/bash
# This is the setup file for Dyse-Robotics
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

helpFunction()
{
   echo ""
   echo -e "\t-e Decides the location of the default python3 environment for this project"
   exit 0 # Exit script after printing help
}

DIR=$PWD
ME="Installer"
echo $DIR

if [[ "$DIR" == *"/Dyse-Robotics" ]]; then
	echo "[${ME}] Initializing Repository" ;
else
	echo $DIR
	echo "[${ME}] Please run this script from the top directory of this repo"; helpFunction ;
fi

while getopts "e:" opt
do
   case "$opt" in
      e ) ENV_PATH="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done


if [ -z "$ENV_PATH" ]; then
	echo -e "[${ME}] Missing default python3 environment path" ;
	helpFunction
fi


python3 -m venv "${ENV_PATH}/dyse-env"
source "${ENV_PATH}/dyse-env/bin/activate"
echo "[${ME}] Creating your default environment"
pip install --ignore-installed -r python3_requirments.txt

if [[ *"${pwd}/dyse-tools"* == "$PYTHONPATH" ]]; then
	echo "[${ME}] Path already exported!"
else
	export PYTHONPATH=$PYTHONPATH:"${pwd}/dyse-tools";
	echo "[${ME}] Added ${PWD}/dyse-tools to PYTHONPATH";
fi

echo "alias load_tools='export PYTHONPATH=$PYTHONPATH:${DIR}/dyse-tools'" >> ~/.bashrc
