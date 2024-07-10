# Required Changes in Autoware
The following changes in Autoware are required in order to work with CARLA and the CARLA-Autoware-Bridge.

## ```autoware_launch``` package
Change the path to the sensor model configuration directory in ```autoware.launch.xml``` to
```xml
<arg name="config_dir" value="$(find-pkg-share carla_t2_sensor_kit_description)/config/"/>
```

Change LiDAR input topic in ```tier4_localization_component.launch.xml``` to
```xml
# input_pointcloud
<arg name="input_pointcloud" default="/sensor/lidar/front" description="The topic will be used in the localization util module"/>
```
