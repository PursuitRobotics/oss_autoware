# Makefile for Autoware Development and Deployment
#
# This Makefile provides a standardized interface for building, testing,
# and deploying the Autoware software stack across different environments (native/baremetal, and docker containers).

# It includes targets for checking prerequisites, setting up the environment,
# building the software, and cleaning up build artifacts.

.PHONY: all help tool-check env-check clean
.DEFAULT_GOAL := help

# --- Shell and Variables ---
SHELL := /bin/bash

# Docker image parameters (can be overridden from the command line)
IMAGE_REGISTRY   ?= ghcr.io/autowarefoundation
IMAGE_NAME       ?= autoware
IMAGE_TAG        ?= 0.1.0
IMAGE_URL        := $(IMAGE_REGISTRY)/$(IMAGE_NAME):$(IMAGE_TAG)

# Semantic Versioning Regex (SemVer 2.0.0)
SEMVER_REGEX := ^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-((?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*)(?:\.(?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*))*))?(?:\+([0-9a-zA-Z-]+(?:\.[0-9a-zA-Z-]+)*))?$$

# --- Colorization ---
NO_COLOR       := \033[0m

RED_LIGHT      := \033[0;31m
RED_BRIGHT     := \033[1;31m
GREEN_LIGHT    := \033[0;32m
GREEN_BRIGHT   := \033[1;32m
BLUE_LIGHT     := \033[0;34m
BLUE_BRIGHT    := \033[1;34m
PURPLE_LIGHT   := \033[0;35m
PURPLE_BRIGHT  := \033[1;35m
ORANGE         := \033[0;33m
WHITE_BRIGHT   := \033[1;37m
GREY           := \033[0;90m

# --- Helper Functions ---
# Defines a function to check for a tool's existence.
# $(call check_tool, <tool_name>)
check_tool = \
    command -v $(1) >/dev/null 2>&1 || \
    (echo -e "$(RED_BRIGHT)Error: Required tool '$(1)' is not installed or not in PATH.$(NO_COLOR)" && exit 1); \
    echo -e "$(GREEN_LIGHT)  [✔] Tool Check: $(1) found at $$(command -v $(1))$(NO_COLOR)"

# --- Main Targets ---

help: ## Display this help menu.
	@echo -e "$(WHITE_BRIGHT)Autoware Project Makefile$(NO_COLOR)"
	@echo -e "$(GREY)Usage: make [target] [VAR=value]$(NO_COLOR)\n"
	@echo -e "$(BLUE_BRIGHT)Environment & Tooling Targets:$(NO_COLOR)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## env-' | awk 'BEGIN {FS = ":.*?## env-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Native (Baremetal) Build Targets:$(NO_COLOR)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## native-' | awk 'BEGIN {FS = ":.*?## native-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Generic Docker Targets (Stubbed):$(NO_COLOR)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## docker-' | awk 'BEGIN {FS = ":.*?## docker-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)OpenAD Kit Docker Build Targets:$(NO_COLOR)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## openad-' | awk 'BEGIN {FS = ":.*?## openad-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Utility Targets:$(NO_COLOR)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## util-' | awk 'BEGIN {FS = ":.*?## util-";}; {printf "  %-30s %s\n", $$1, $$2}'

# ==============================================================================
# Environment & Tooling Targets
# ==============================================================================

tool-check: ## env-Verify that essential tools (docker, git) are installed.
	@echo -e "$(PURPLE_BRIGHT)--- Checking for required tools ---$(NO_COLOR)"
	@$(call check_tool,docker)
	@$(call check_tool,git)

env-check: tool-check ## env-Run all environment and dependency checks.
	@echo -e "$(PURPLE_BRIGHT)--- Checking OS and dependencies ---$(NO_COLOR)"
	@if ! grep -q 'Ubuntu 22.04' /etc/os-release; then \
		echo -e "$(RED_BRIGHT)Error: Host OS is not Ubuntu 22.04. Some features may not work.$(NO_COLOR)"; \
		exit 1; \
	fi
	@echo -e "$(GREEN_LIGHT)  [✔] OS Check: Ubuntu 22.04 detected.$(NO_COLOR)"
	@if ! dpkg -s ros-dev-tools >/dev/null 2>&1; then \
		echo -e "$(RED_BRIGHT)Error: 'ros-dev-tools' is not installed. Please run 'sudo apt install ros-dev-tools'.$(NO_COLOR)"; \
		exit 1; \
	fi
	@echo -e "$(GREEN_LIGHT)  [✔] Dependency Check: ros-dev-tools is installed.$(NO_COLOR)"
	@if ! command -v vcstool >/dev/null 2>&1; then \
		echo -e "$(RED_BRIGHT)Error: 'vcstool' is not installed. Please run 'sudo apt install python3-vcstool'.$(NO_COLOR)"; \
		exit 1; \
	fi
	@echo -e "$(GREEN_LIGHT)  [✔] Dependency Check: vcstool is installed.$(NO_COLOR)"

_check-semver:
	@if ! [[ "$(IMAGE_TAG)" =~ $(SEMVER_REGEX) ]]; then \
		echo -e "$(RED_BRIGHT)Error: IMAGE_TAG '$(IMAGE_TAG)' is not a valid Semantic Version.$(NO_COLOR)"; \
		exit 1; \
	fi

# ==============================================================================
# Native (Baremetal) Build Targets
# ==============================================================================

native-setup: env-check ## native-Run the development environment setup script (setup-dev-env.sh).
	@echo -e "$(PURPLE_BRIGHT)--- Setting up Native Development Environment ---$(NO_COLOR)"
	@echo "This will run '../setup-dev-env.sh'. It may ask for your sudo password."
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		../setup-dev-env.sh; \
	else \
		echo -e "$(ORANGE)Setup cancelled.$(NO_COLOR)"; \
	fi

native-import: env-check ## native-Import Autoware source repositories using vcstool.
	@echo -e "$(PURPLE_BRIGHT)--- Importing source repositories ---$(NO_COLOR)"
	mkdir -p ../src
	vcs import ../src < ../autoware.repos

native-deps: env-check ## native-Install system dependencies using rosdep.
	@echo -e "$(PURPLE_BRIGHT)--- Installing ROS dependencies ---$(NO_COLOR)"
	rosdep update
	rosdep install -y --from-paths ../src --ignore-src --rosdistro humble

native-build: env-check ## native-Build the Autoware workspace using colcon.
	@echo -e "$(PURPLE_BRIGHT)--- Building Autoware workspace ---$(NO_COLOR)"
	source /opt/ros/humble/setup.bash
	cd .. && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

native-clean: ## native-Clean the native build artifacts (build, install, log).
	@echo -e "$(PURPLE_BRIGHT)--- Cleaning native workspace ---$(NO_COLOR)"
	rm -rf ../build/ ../install/ ../log/

# ==============================================================================
# Generic Docker Targets (Stubbed)
# ==============================================================================

docker-build: tool-check _check-semver ## docker-Build a Docker image (stubbed). Params: IMAGE_URL, IMAGE_NAME, IMAGE_TAG.
	@echo -e "$(PURPLE_BRIGHT)--- Building Docker Image (Stub) ---$(NO_COLOR)"
	@echo -e "Image Name: $(ORANGE)$(IMAGE_NAME)$(NO_COLOR)"
	@echo -e "Image Tag:  $(ORANGE)$(IMAGE_TAG)$(NO_COLOR)"
	@echo -e "Full URL:   $(ORANGE)$(IMAGE_URL)$(NO_COLOR)"
	@echo -e "$(GREY)Command: docker build -t $(IMAGE_URL) .$(NO_COLOR)"

docker-tag: tool-check _check-semver ## docker-Tag a Docker image (stubbed). Params: IMAGE_URL, IMAGE_NAME, IMAGE_TAG.
	@echo -e "$(PURPLE_BRIGHT)--- Tagging Docker Image (Stub) ---$(NO_COLOR)"
	@echo -e "Source:     $(ORANGE)$(IMAGE_NAME):latest$(NO_COLOR)"
	@echo -e "Target:     $(ORANGE)$(IMAGE_URL)$(NO_COLOR)"
	@echo -e "$(GREY)Command: docker tag $(IMAGE_NAME):latest $(IMAGE_URL)$(NO_COLOR)"

docker-push: tool-check _check-semver ## docker-Push a Docker image to a registry (stubbed). Params: IMAGE_URL.
	@echo -e "$(PURPLE_BRIGHT)--- Pushing Docker Image (Stub) ---$(NO_COLOR)"
	@echo -e "Pushing to: $(ORANGE)$(IMAGE_URL)$(NO_COLOR)"
	@echo -e "$(GREY)Command: docker push $(IMAGE_URL)$(NO_COLOR)"

# ==============================================================================
# Docker Build Environments
# ==============================================================================

docker-monolithic-build: ## openad-Build a single, monolithic Docker container (stubbed).
	@echo -e "$(PURPLE_BRIGHT)--- Building Monolithic Docker Image (Stub) ---$(NO_COLOR)"
	@echo -e "$(GREY)This target is a placeholder. It would typically run a command like:$(NO_COLOR)"
	@echo -e "$(GREY)docker build -f ../docker/Dockerfile.monolithic -t $(IMAGE_REGISTRY)/$(IMAGE_NAME):$(IMAGE_TAG)-monolithic .$(NO_COLOR)"
	@echo -e "$(ORANGE)Action: No operation performed.$(NO_COLOR)"

docker-openad-build: env-check ## openad-Build non-CUDA images using the OpenAD Kit 'docker bake' prescription.
	@echo -e "$(PURPLE_BRIGHT)--- Building OpenAD Kit Images (non-CUDA) ---$(NO_COLOR)"
	@echo "This will build all non-CUDA targets defined in '../docker/docker-bake.hcl'."
	@echo -e "$(GREY)Command: docker bake -f ../docker/docker-bake.hcl --progress=plain$(NO_COLOR)"
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker bake -f ../docker/docker-bake.hcl --progress=plain; \
	else \
		echo -e "$(ORANGE)Build cancelled.$(NO_COLOR)"; \
	fi

docker-openad-build-cuda: env-check ## openad-Build CUDA-enabled images using the OpenAD Kit 'docker bake' prescription.
	@echo -e "$(PURPLE_BRIGHT)--- Building OpenAD Kit Images (CUDA) ---$(NO_COLOR)"
	@echo "This will build all CUDA targets defined in '../docker/docker-bake-cuda.hcl'."
	@echo -e "$(GREY)Command: docker bake -f ../docker/docker-bake-cuda.hcl --progress=plain$(NO_COLOR)"
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker bake -f ../docker/docker-bake-cuda.hcl --progress=plain; \
	else \
		echo -e "$(ORANGE)Build cancelled.$(NO_COLOR)"; \
	fi

# ==============================================================================
# Utility Targets
# ==============================================================================

clean: native-clean ## util-Clean all build artifacts from the workspace.
	@echo -e "$(PURPLE_BRIGHT)--- Cleaning Docker artifacts (pruning dangling images) ---$(NO_COLOR)"
	-docker image prune -f