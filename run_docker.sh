#!/bin/bash

# Acknowledgement
#
# Special thanks to GEONMO YANG(ygm7422@gmail.com)
# for providing the initial shell script and docker-compose configuration,
# which played a key role in setting up the Docker-based environment for this project.

# $$ : shell process number (pid)
# $0 : shell script name
# $1 : dataset path

to_docker_path() {
    local RELATIVE_PATH=$(echo "`dirname docker/docker-compose.yml`")
    echo $(realpath $RELATIVE_PATH)
}

to_workspace_path() {
    local RELATIVE_PATH=$(echo "`dirname $DOCKER_PATH`")
    echo $(realpath $RELATIVE_PATH)
}

# Check if dataset path argument is provided
if [ -z $1 ]; then
    echo "Error: Dataset path is required."
    echo "Usage: $0 <dataset_path>"
    exit 1
fi

DOCKER_PATH=$(to_docker_path)
WORKSPACE_PATH=$(to_workspace_path)
export WORKSPACE_NAME=$(echo "`basename $WORKSPACE_PATH`")
export BASE_PATH=$(dirname "$(dirname "$(dirname "$DOCKER_PATH")")")

SERVICE=${WORKSPACE_NAME}            # service name

export DATASET_PATH=$1                            # dataset path define
export DOCKER_FILE=open_lmm.Dockerfile
export DOCKER_IMAGE=hwan0806/${WORKSPACE_NAME}:latest

xhost +local:docker

docker compose --file ${DOCKER_PATH}/docker-compose.yml up -d ${SERVICE}
