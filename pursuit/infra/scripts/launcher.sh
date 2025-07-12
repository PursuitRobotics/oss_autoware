#!/usr/bin/env bash

# ==============================================================================
# build_launcher.sh
#
# Description:
#   This script cleans specified package directories from the 'build', 'install',
#   and 'log' directories of the project. It is designed to be run from its
#   location within 'pursuit/infra/scripts'.
#
# Usage:
#   ./build_launcher.sh
#
# ==============================================================================

# --- Script Configuration ---

# Exit immediately if a command exits with a non-zero status.
set -e
# Treat unset variables as an error when substituting.
set -u
# Pipes return the exit status of the last command to exit with a non-zero status.
set -o pipefail

# --- Directory and Path Setup ---

# Get the absolute path of the directory where the script is located.
# This makes the script runnable from any location.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# The project root is two levels up from the script's directory (scripts -> infra -> root).
PROJECT_ROOT="$SCRIPT_DIR/../.."

# --- Main Logic ---

# An array of package names to be removed.
# This makes the list easy to manage and prevents repetition.
PACKAGES_TO_CLEAN=(
    "ugv1_common_launch"
    "ugv1_interface"
    "ugv1_sensor_kit_description"
    "ugv1_sensor_kit_launch"
    "ugv1_vehicle_description"
    "ugv1_vehicle_launch"
)

# An array of top-level directories to clean.
TARGET_DIRS=(
    "build"
    "install"
    "log"
)

echo "Starting cleanup process for project at: $PROJECT_ROOT"
echo "--------------------------------------------------------"

# Loop through each target directory ('build', 'install', 'log').
for dir in "${TARGET_DIRS[@]}"; do
    TARGET_DIR_PATH="$PROJECT_ROOT/$dir"

    echo
    echo "Processing directory: $TARGET_DIR_PATH"

    # Check if the target directory exists before trying to clean it.
    if [ ! -d "$TARGET_DIR_PATH" ]; then
        echo "  -> Directory not found. Skipping."
        continue
    fi

    # Loop through each package and remove its corresponding directory.
    for pkg in "${PACKAGES_TO_CLEAN[@]}"; do
        PKG_PATH="$TARGET_DIR_PATH/$pkg"

        # Check if the specific package directory exists before trying to remove it.
        if [ -d "$PKG_PATH" ]; then
            echo "  -> Removing: $PKG_PATH"
            # Use rm -rf to force-remove the directory and its contents.
            rm -rf "$PKG_PATH"
        else
            echo "  -> Already removed (not found): $PKG_PATH"
        fi
    done
done

echo
echo "--------------------------------------------------------"
echo "Cleanup complete."
