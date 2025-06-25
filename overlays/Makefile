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

# Tool paths
TOOL_GIT_PATH := $(shell command -v git)
TOOL_DOCKER_PATH := $(shell command -v docker)
TOOL_VCS_PATH := $(shell command -v vcs)
TOOL_ROSDEP_PATH := $(shell command -v rosdep)


# Docker image parameters (can be overridden from the command line)
IMAGE_REGISTRY   ?= ghcr.io/autowarefoundation
IMAGE_NAME       ?= autoware
IMAGE_TAG        ?= 0.1.0
IMAGE_URL        := $(IMAGE_REGISTRY)/$(IMAGE_NAME):$(IMAGE_TAG)

# Semantic Versioning Regex (SemVer 2.0.0)
SEMVER_REGEX := ^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-((?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*)(?:\.(?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*))*))?(?:\+([0-9a-zA-Z-]+(?:\.[0-9a-zA-Z-]+)*))?$$

# --- Colorization ---
RESET       := \033[0m

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
    (echo -e "$(RED_BRIGHT)Error: Required tool '$(1)' is not installed or not in PATH.$(RESET)" && exit 1); \
    echo -e "$(GREEN_LIGHT)  [✔] Tool Check: $(1) found at $$(command -v $(1))$(RESET)"

# --- Main Targets ---

help: ## Display this help menu.
	@echo -e "$(WHITE_BRIGHT)Autoware Project Makefile$(RESET)"
	@echo -e "$(GREY)Usage: make [target] [VAR=value]$(RESET)\n"
	@echo -e "$(BLUE_BRIGHT)Environment & Tooling Targets:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## env-' | awk -v T="$(GREEN_BRIGHT)" -v D="$(GREY)" -v R="$(RESET)" 'BEGIN {FS = ":.*?## env-";}; {printf T "  %-30s" R " " D "%s" R "\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Native (Baremetal) Build Targets:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## native-' | awk 'BEGIN {FS = ":.*?## native-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Generic Docker Targets (Stubbed):$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## docker-' | awk 'BEGIN {FS = ":.*?## docker-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)OpenAD Kit Docker Build Targets:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## openad-' | awk 'BEGIN {FS = ":.*?## openad-";}; {printf "  %-30s %s\n", $$1, $$2}'
	@echo -e "\n$(BLUE_BRIGHT)Utility Targets:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep '## util-' | awk 'BEGIN {FS = ":.*?## util-";}; {printf "  %-30s %s\n", $$1, $$2}'

# ==============================================================================
# Environment & Tooling Targets
# ==============================================================================

tool-check: ## env-Verify that essential tools (docker, git) are installed.
	@echo -e "$(PURPLE_BRIGHT)--- Checking for required tools ---$(RESET)"
	@$(call check_tool,docker)
	@$(call check_tool,git)

env-check: tool-check ## env-Run all environment and dependency checks.
	@echo -e "$(PURPLE_BRIGHT)--- Checking OS envars, and dependencies ---$(RESET)"
	@if ! grep -q 'Ubuntu 22.04' /etc/os-release; then \
		echo -e "$(RED_BRIGHT)Error: Host OS is not Ubuntu 22.04. Some features may not work.$(RESET)"; \
		exit 1; \
	fi
	@echo -e "$(GREY)  [✔] OS Check: Ubuntu 22.04 detected.$(RESET)"
	@if [ -z "$(TOOL_ROSDEP_PATH)" ]; then \
		echo -e "$(RED_BRIGHT)Error: 'rosdep' is not found in PATH. Please run 'sudo apt install python3-rosdep'.$(RESET)"; \
		exit 1; \
	fi
	@echo -e "$(GREEN_LIGHT)  [✔] Dependency Check: rosdep is installed at $(TOOL_ROSDEP_PATH).$(RESET)"
	@if [ -z "$(TOOL_VCS_PATH)" ]; then \
		echo -e "$(RED_BRIGHT)Error: 'vcs' (from vcstool) is not found in PATH. Please run 'sudo apt install python3-vcstool'.$(RESET)"; \
		exit 1; \
	fi
	@echo -e "$(GREEN_LIGHT)  [✔] Dependency Check: vcs (from vcstool) is installed at $(TOOL_VCS_PATH).$(RESET)"

_check-semver:
	@if ! [[ "$(IMAGE_TAG)" =~ $(SEMVER_REGEX) ]]; then \
		echo -e "$(RED_BRIGHT)Error: IMAGE_TAG '$(IMAGE_TAG)' is not a valid Semantic Version.$(RESET)"; \
		exit 1; \
	fi

# ==============================================================================
# Native (Baremetal) Build Targets
# ==============================================================================

native-setup: env-check ## native-Run the development environment setup script (setup-dev-env.sh).
	@echo -e "$(PURPLE_BRIGHT)--- Setting up Native Development Environment --> No CUDA/NVIDIA---$(RESET)"
	@echo "This will run '../setup-dev-env.sh'. It may ask for your sudo password."
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		(cd .. && ./setup-dev-env.sh --no-nvidia --no-cuda-drivers --runtime); \
	else \
		echo -e "$(ORANGE)Setup cancelled.$(RESET)"; \
	fi

native-import: env-check ## native-Import Autoware source repositories using vcstool.
	@echo -e "$(PURPLE_BRIGHT)--- Importing source repositories ---$(RESET)"
	mkdir -p ../src
	vcs import ../src < ../autoware.repos

native-deps: env-check ## native-Install system dependencies using rosdep.
	@echo -e "$(PURPLE_BRIGHT)--- Installing ROS dependencies ---$(RESET)"
	rosdep update
	rosdep install -y --from-paths ../src --ignore-src --rosdistro humble

native-build: env-check ## native-Build the Autoware workspace using colcon.
	@echo -e "$(PURPLE_BRIGHT)--- Building Autoware workspace ---$(RESET)"
	source /opt/ros/humble/setup.bash
	(cd .. && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)

native-build-clean: env-check native-clean ## native-Build the Autoware workspace using colcon.
	@echo -e "$(PURPLE_BRIGHT)--- Building Autoware workspace ---$(RESET)"
	source /opt/ros/humble/setup.bash
	(cd .. && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)


native-clean: ## native-Clean the native build artifacts (build, install, log).
	@echo -e "$(PURPLE_BRIGHT)--- Cleaning native workspace ---$(RESET)"
	rm -rf ../build/ ../install/ ../log/

# ==============================================================================
# Generic Docker Targets (Stubbed)
# ==============================================================================

docker-build: tool-check _check-semver ## docker-Build a Docker image (stubbed). Params: IMAGE_URL, IMAGE_NAME, IMAGE_TAG.
	@echo -e "$(PURPLE_BRIGHT)--- Building Docker Image (Stub) ---$(RESET)"
	@echo -e "Image Name: $(ORANGE)$(IMAGE_NAME)$(RESET)"
	@echo -e "Image Tag:  $(ORANGE)$(IMAGE_TAG)$(RESET)"
	@echo -e "Full URL:   $(ORANGE)$(IMAGE_URL)$(RESET)"
	@echo -e "$(GREY)Command: docker build -t $(IMAGE_URL) .$(RESET)"

docker-tag: tool-check _check-semver ## docker-Tag a Docker image (stubbed). Params: IMAGE_URL, IMAGE_NAME, IMAGE_TAG.
	@echo -e "$(PURPLE_BRIGHT)--- Tagging Docker Image (Stub) ---$(RESET)"
	@echo -e "Source:     $(ORANGE)$(IMAGE_NAME):latest$(RESET)"
	@echo -e "Target:     $(ORANGE)$(IMAGE_URL)$(RESET)"
	@echo -e "$(GREY)Command: docker tag $(IMAGE_NAME):latest $(IMAGE_URL)$(RESET)"

docker-push: tool-check _check-semver ## docker-Push a Docker image to a registry (stubbed). Params: IMAGE_URL.
	@echo -e "$(PURPLE_BRIGHT)--- Pushing Docker Image (Stub) ---$(RESET)"
	@echo -e "Pushing to: $(ORANGE)$(IMAGE_URL)$(RESET)"
	@echo -e "$(GREY)Command: docker push $(IMAGE_URL)$(RESET)"

# ==============================================================================
# Docker Build Environments
# ==============================================================================

docker-monolithic-build: ## openad-Build a single, monolithic Docker container (stubbed).
	@echo -e "$(PURPLE_BRIGHT)--- Building Monolithic Docker Image (Stub) ---$(RESET)"
	@echo -e "$(GREY)This target is a placeholder. It would typically run a command like:$(RESET)"
	@echo -e "$(GREY)docker build -f ../docker/Dockerfile.monolithic -t $(IMAGE_REGISTRY)/$(IMAGE_NAME):$(IMAGE_TAG)-monolithic .$(RESET)"
	@echo -e "$(ORANGE)Action: No operation performed.$(RESET)"

docker-openad-build: env-check ## openad-Build non-CUDA images using the OpenAD Kit 'docker bake' prescription.
	@echo -e "$(PURPLE_BRIGHT)--- Building OpenAD Kit Images (non-CUDA) ---$(RESET)"
	@echo "This will build all non-CUDA targets defined in '../docker/docker-bake.hcl'."
	@echo -e "$(GREY)Command: docker bake -f ../docker/docker-bake.hcl --progress=plain$(RESET)"
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker bake -f ../docker/docker-bake.hcl --progress=plain; \
	else \
		echo -e "$(ORANGE)Build cancelled.$(RESET)"; \
	fi

docker-openad-build-cuda: env-check ## openad-Build CUDA-enabled images using the OpenAD Kit 'docker bake' prescription.
	@echo -e "$(PURPLE_BRIGHT)--- Building OpenAD Kit Images (CUDA) ---$(RESET)"
	@echo "This will build all CUDA targets defined in '../docker/docker-bake-cuda.hcl'."
	@echo -e "$(GREY)Command: docker bake -f ../docker/docker-bake-cuda.hcl --progress=plain$(RESET)"
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker bake -f ../docker/docker-bake-cuda.hcl --progress=plain; \
	else \
		echo -e "$(ORANGE)Build cancelled.$(RESET)"; \
	fi

# ==============================================================================
# Utility Targets
# ==============================================================================

clean: native-clean ## util-Clean all build artifacts from the workspace.
	@echo -e "$(PURPLE_BRIGHT)--- Cleaning Docker artifacts (pruning dangling images) ---$(RESET)"
	-docker image prune -f