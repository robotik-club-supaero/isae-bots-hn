# Makefile for ISAEBOTS development
#
# Usage:
# 	make <target> <arg>=<val>
# 	
#
# Intallation:
#	make build-core
# 	make build-base
#	make create-container
#	make main

# Detection of architecture (PC or pi)
architecture = $(shell uname -m)

# Docker intern variables
IMAGE_NAME = isaebots_desktop_env
IMAGE_NAME_PI = isaebots_pi_env_full
CONTAINER_NAME = isaebots
PS_ID = null
CMD = bash
CORE_DOCKERFILE = ${PWD}/docker/dockerfile.core
BASE_DOCKERFILE = ${PWD}/docker/dockerfile.base
PI_DOCKERFILE = ${PWD}/docker/dockerfile.pi
PI_PLATFORM = linux/arm64/v8

# Setup Docker volumes and env variables
DOCKER_VOLUMES = \
	--volume="${PWD}/dev":"/app/dev" \
	--volume="/dev":"/dev" \
	--volume="${PWD}/scripts":"/app/scripts" \
	--volume="/tmp/.X11-unix":"/tmp/.X11-unix"


DOCKER_VOLUMES_PI = \
	--volume="${PWD}/dev":"/app/dev" \
	--volume="/dev":"/dev" \
	--volume="${PWD}/scripts":"/app/scripts"


DOCKER_ENV_VAR = \
	-e DISPLAY=${DISPLAY} \
	--env="WDIR=dev"

docker_ip = $(shell ip -4 -o a| grep docker0 | awk '{print $4}' | cut -d/ -f1)

DOCKER_ENV_VAR_PI = \
	--env="WDIR=dev" \
	--env PULSE_SERVER=tcp:${docker_ip}:34567

# --env PULSE_SERVER=tcp:172.17.0.1:34567 fait marcher pulseaudio dans le docker sur la pi pour avoir du son en TCP
# ref du tuto : https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

# .PHONY means that the Makefile command doesn't use any file as a source
.PHONY: help
help:
	@echo "=== HELP message ===================================="
	@echo "make build-core    for Ubuntu-20.04 + ROS-noetic img "
	@echo "make build-base    for Desktop ISAEBOTS dev env img  "
	@echo ""
	@echo "make kill          to kill any running env container "
	@echo "make term          to init a terminal in env Desktop "
	@echo "====================================================="

#############################################################
# SETUP
#############################################################

test:
	@echo ${CORE_DOCKERFILE} ${IMAGE_NAME}

.PHONY: print-architecture
print-architecture:
	@if [ "${architecture}" = "x86_64" ]; then \
		echo "The architecture is $(architecture), we are on the PC"; \
	elif [ "${architecture}" = "aarch64" ]; then \
		echo "The architecture is $(architecture), we are on the pi"; \
	else \
		echo "The architecture is $(architecture), wtf is that"; \
	fi


# Build the core image
.PHONY: build-core
build-core:
	@echo ${CORE_DOCKERFILE} ${IMAGE_NAME}
	@docker build -f ${CORE_DOCKERFILE} -t ${IMAGE_NAME}_core .

# Build the base image (depends on core image build)
.PHONY: build-base
build-base: build-core
	@docker build -f ${BASE_DOCKERFILE} -t ${IMAGE_NAME}_base .

# Build the image for the raspberry pi (64 bits so architecture linux/arm64/v8)
# To be able to do that, docker buildx needs to be installed (cf tuto installs on new pi)
.PHONY: build-image-pi
build-image-pi:
	@docker buildx build --platform=${PI_PLATFORM} -f ${PI_DOCKERFILE} -t ${IMAGE_NAME_PI} . --load


.PHONY: create-container
create-container:

#	the 'privileged' flag is necessary ? might be so that we get access to the GPIO and other stuff
#	we should log in as a user and not root (preferable)

#	Check if container has been created, if not create it
	@if [ -z $$(docker ps -aqf name=$(CONTAINER_NAME)) ]; then \
        echo "Creating container $(CONTAINER_NAME) ..."; \
		if [ "${architecture}" = "x86_64" ]; then \
			docker run -it --privileged --net=host \
			--name ${CONTAINER_NAME} \
			${DOCKER_VOLUMES} \
			${DOCKER_ENV_VAR} \
			-u dockeruser \
			${IMAGE_NAME}_base \
			"${CMD}"; \
			echo "Created PC container successfully"; \
		elif [ "${architecture}" = "aarch64" ]; then \
			docker run -it --privileged --net=host \
			--name ${CONTAINER_NAME} \
			${DOCKER_VOLUMES_PI} \
			${DOCKER_ENV_VAR_PI} \
			-u dockeruser \
			${IMAGE_NAME_PI} \
			"${CMD}"; \
			echo "Created pi container successfully"; \
		else \
			echo "The architecture is $(architecture), not recognized"; \
		fi \
    else \
        echo "Container $(CONTAINER_NAME) is already created"; \
    fi

#############################################################
# TASKS
#############################################################

# Kill any running Docker containers
#.PHONY: kill
# /!\ doesn't kill a running container, only stopped containers (to do it use docker kill $(docker container ls -q)
#kill: 
#	@echo "Closing already running container"
#	@docker container prune -f
	

# Removes the container before running it again to make a new one
.PHONY: clear-container
clear-container:
	@if [ -z $$(docker ps -aqf name=$(CONTAINER_NAME)) ]; then \
		echo "Container $(CONTAINER_NAME) doesn't exist yet"; \
	else \
		echo "Replacing container $(CONTAINER_NAME) with a new one ..."; \
		docker container rm $(CONTAINER_NAME) > /dev/null; \
		if [ "${architecture}" = "x86_64" ]; then \
			docker run -it --privileged --net=host \
			--name ${CONTAINER_NAME} \
			${DOCKER_VOLUMES} \
			${DOCKER_ENV_VAR} \
			-u dockeruser \
			${IMAGE_NAME}_base \
			"${CMD}"; \
			echo "Replaced PC container successfully"; \
		elif [ "${architecture}" = "aarch64" ]; then \
			docker run -it --privileged --net=host \
			--name ${CONTAINER_NAME} \
			${DOCKER_VOLUMES_PI} \
			${DOCKER_ENV_VAR_PI} \
			-u dockeruser \
			${IMAGE_NAME_PI} \
			"${CMD}"; \
			echo "Replaced pi container successfully"; \
		else \
			echo "The architecture is $(architecture), not recognized"; \
		fi \
	fi


# Start a terminal inside the Docker container, and then close the container (difference with make term)
.PHONY: main
main: create-container
#	Check if container is running
	@if [ -z $$(docker ps -qf name=$(CONTAINER_NAME)) ]; then \
        echo "Starting container $(CONTAINER_NAME) ..."; \
		docker start $(CONTAINER_NAME) > /dev/null; \
    else \
        echo "Container $(CONTAINER_NAME) is already running"; \
    fi

	@docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/noetic/setup.bash; ${CMD}"

	@echo "Stopping container $(CONTAINER_NAME) ..."
	@docker kill $(CONTAINER_NAME) > /dev/null;



# Start a terminal inside the Docker container, doesn't close it at on exit
.PHONY: term
term:
#	Check if container is running
	@if [ -z $$(docker ps -qf name=$(CONTAINER_NAME)) ]; then \
        echo "Container $(CONTAINER_NAME) is not started yet"; \
    else \
        docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/noetic/setup.bash; ${CMD}"; \
    fi

	
# Terminal used for the simulation with special bindkeys
.PHONY: sim_term
sim_term:
	@docker exec -it $(shell docker ps -aqf "name=${CONTAINER_NAME}") bash --rcfile ./dev/src/uix/log/simTerm_rc.sh


# Running container called NAME :
# docker ps -aqf status=running --filter name=NAME
# OR
# docker ps -qf name=NAME

# Stopped container called NAME
# docker ps -aqf status=exited --filter name=NAME
