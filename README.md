# ROS2 Demos

This repository contains ROS2 demo programs using pub/sub to share a generic
IoT data type. The demos can be build by:
```
ament build
```
The individual packages are under the `src` subdirectory:

- iot_msgs Definition of generic IoT data type and helper functions

- iot_publisher Excecutable publisher of IoT data samples

- iot_subscriber Excecuable subscriber of IoT data samples
