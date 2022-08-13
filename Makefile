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
PS_NAME = dev_ps
PS_ID = null
CMD = bash
CORE_DOCKERFILE = ${PWD}/docker/dockerfile.core
BASE_DOCKERFILE = ${PWD}/docker/dockerfile.base

# Setup Docker volumes and env variables
DOCKER_VOLUMES = \
	--volume="${PWD}/dev":"/app/dev" 
#	--volume="${PWD}/doc":"/app/doc" \
#	--volume="${PWD}/scripts":"/app/scripts"

DOCKER_ENV_VAR = \
	--env="DISPLAY" \
	--env="WDIR=dev"

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

# Build the core image
.PHONY: build-core
build-core:
	@echo ${CORE_DOCKERFILE} ${IMAGE_NAME}
	@docker build -f ${CORE_DOCKERFILE} -t ${IMAGE_NAME}_core .

# Build the base image (depends on core image build)
.PHONY: build-base
build-base: build-core
	@docker build -f ${BASE_DOCKERFILE} -t ${IMAGE_NAME}_base .


#############################################################
# TASKS
#############################################################

# Kill any running Docker containers
.PHONY: kill
kill: 
	@echo "Closing already running container"
	@docker container prune -f
	
# Start a terminal inside the Docker container
.PHONY: main
main:# kill
	@docker run --rm -it --net=host \
		--name ${PS_NAME} \
		${DOCKER_VOLUMES} \
		${DOCKER_ENV_VAR} \
		${IMAGE_NAME}_base \
		${CMD}

.PHONY: term
term:
	@docker exec -it $(shell docker ps -aqf "name=${PS_NAME}") bash -c "${CMD}"

.PHONY: log_term
log_term:
	@docker exec -it $(shell docker ps -aqf "name=${PS_NAME}") bash -c "source /opt/ros/noetic/setup.bash && ${CMD}; bash"