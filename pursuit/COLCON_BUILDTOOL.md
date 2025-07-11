# Colcon Build Tool (`colcon build`)

`colcon` is a command line tool to build, test, and deploy software packages. The `build` command is used to build a set of packages. This document outlines the key arguments for the `colcon build` command.

## Key Arguments

| Argument | Description |
|---|---|
| `--packages-up-to` | Build the specified packages and all their dependencies. This is useful for building a specific package and all the packages it depends on. |
| `--packages-select` | Build only the specified packages. This is useful for building a subset of packages. |
| `--symlink-install` | Use symlinks instead of copying files. This can speed up the build process, as it avoids copying large files. |
| `--cmake-args` | Pass arguments to the CMake build system. This is useful for setting CMake variables, such as `CMAKE_BUILD_TYPE`. |
| `--ament-cmake-args` | Pass arguments to the Ament CMake build system. This is useful for setting Ament-specific CMake variables. |
| `--event-handlers` | Specify event handlers to use. This is useful for customizing the output of the build process. For example, you can use the `console_direct+` event handler to get real-time output from the build process. |
| `--mixin` | Use a mixin to extend the build process. Mixins are a way to add custom build steps to the build process. |
