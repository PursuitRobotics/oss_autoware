xfce (minimal x-windows desktop)
then install vnc ; tigervnc (setup with particular windowing system)

# Planning Simulation


```shell
source install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml map_path:=${AW_MAP}/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit perception:=false

```
# Rosbag Simulator

ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-rosbag vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit perception:=false

ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-rosbag vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit perception:=false rviz:=false

# need to download the data for sample-rosbag
ros2 bag play $HOME/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3

ros2 bag info $AW_MAP/sample-rosbag

ros2 topic list

ros2 topic echo <topic>

# Scenario Testing


```shell
source install/setup.bash

ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/universe/20250130 \
  record:=false \
  scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle
```

# Random Test Simulation


```shell
source install/setup.bash

ros2 launch random_test_runner random_test.launch.py \
  architecture_type:=awf/universe/20250130 \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle

```

# Driving Log Replayer 

https://tier4.github.io/driving_log_replayer/quick_start/installation/


# Install GUI 

```bash

VERSION=v1.0.5
sudo apt install ./autoware-launch-gui_${VERSION}_amd64.deb


git clone https://github.com/leo-drive/autoware-launch-gui.git

```pnpm tauri dev
