# ROS2 Core Documentations

## Useful Resources

### ROS2 Tutorials

- https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html
- https://mcap.dev/guides/python/protobuf

#### Create Package


- https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- 

```bash

PKG_NAME=jdev
PKG_NODE=hello-world

# go to your workspace; eg; cd ~/autoware_ws/src

ros2 pkg create --build-type ament_cmake --node-name $PKG_NODE $PKG_NAME

cd ..
source ./install/local_setup.bash
colcon build 
# or
colcon build --packages-select ${PKG_NAME}

# to run the node
# make sure the install is sourced.
ros2 run $PKG_NAME $PKG_NODE

```


### ROS1 Tutorials

- https://docs.ros.org/en/rolling/Tutorials.html


GitHub

Monitoring Solutions

- https://github.com/baseonballs/benchmarks
- https://docs.ros.org/en/rolling/Concepts.html
- https://github.com/Haivision/srt-prometheus-exporter
- https://discourse.ros.org/t/project-proposal-ros2-tool-to-collect-topic-metrics/32790/6
- https://discourse.ros.org/tag/metrics



- https://docs.vulcanexus.org/en/iron/rst/tutorials/tools/prometheus/prometheus.html
- cd .