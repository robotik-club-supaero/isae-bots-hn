# Makefile for ISAEBOTS development
#
# Usage:
# 	make <target> <arg>=<val>
# 	
#
# Demos:
#	make build-core
# 	make build-base

# Docker intern variables
IMAGE_NAME = isaebots_desktop_env
CORE_DOCKERFILE = ${PWD}/docker/dockerfile.core
BASE_DOCKERFILE = ${PWD}/docker/dockerfile.base

# Setup Docker volumes and env variables
DOCKER_VOLUMES = \
	--volume="${PWD}/dev/hn":"/desktop_ws":rw

DOCKER_VAR_ENV = 

DOCKER_ARGS = ${DOCKER_VOLUMES} ${DOCKER_VAR_ENV}

.PHONY: help
help:
	@echo "help"

#############################################################
# SETUP
#############################################################

# Build the core image
.PHONY: build-core
build-core:
	@docker build -f ${CORE_DOCKERFILE} -t ${IMAGE_NAME}_core .

# Build the base image (depends on core image build)
.PHONY: build-base
build-base: build-core
	@docker build -f ${BASE_DOCKERFILE} -t ${IMAGE_NAME}_base .

# Kill any running Docker containers
.PHONY: kill
kill: 
	@echo "Closing all running Docker containers:"
	@docker kill $(shell docker ps -q --filter ancestor=${IMAGE_NAME}_base)

#############################################################
# TASKS
#############################################################

# Start a terminal inside the Docker container
.PHONY: term
term: 
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_base bash