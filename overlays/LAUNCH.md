

# Planning Simulation


```shell
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=${AW_MAP}/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

```

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


# Install GUI 

```bash

VERSION=v1.0.5
sudo apt install ./autoware-launch-gui_${VERSION}_amd64.deb


git clone https://github.com/leo-drive/autoware-launch-gui.git
```