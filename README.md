# ros2_hyperion_interrogator
Interrogator for FBG Interrogator Hyperion x55

## Nodes
There are three different nodes here for the interrogator and nodes for calling the interrogator services.

### `hyperion_talker`
The default interrogator to interface with the six55 Hyperion interrogators. This has slow peak streaming, but is stable.

#### Parameters
* interrogator.ip_address: the IP address of the interrogator. Used for reconnect service.
* sensor.num_samples: the number of peak samples to gather when calibrating the sensors. 

#### Publishers
* /sensor/raw: std_msgs/msg/Float64MultiArray of raw peak signals for all channels
* /sensor/CHX/raw: std_msgs/msg/Float64MultiArray of raw peak signals per channel
* /sensor/CHX/processed: std_msgs/msg/Float64MultiArray of processed peak signals per channel
* /sensor/processed: std_msgs/msg/Float64MultiArray of processed peak signals for all channels

#### Subscribers
None

#### Services
* /interrogator/reconnect: std_srvs/srv/Trigger, triggers the node to reconnect to the interrogator after grabbing the IP address parameter
* /sensor/calibrate: std_srvs/srv/Trigger, triggers the node to grab a specified number of peak signals to determine a baseline for the base Bragg wavelength for processing.
  After the sensors are calibrated. Then the node will publish on the processed topics.

### `hyperion_streamer`
The peak streaming node to quickly stream peaks from the six55 Hyperion interrogators. This is very quick and has been tested to work, but requires `asyncio`.

#### Parameters
Same as `hyperion_talker`.

#### Publishers
Same as `hyperion_talker`.

#### Subscibers
Same as `hyperion_talker`.

#### Services
Same as `hyperion_talker` except without /interrogator/reconnect. This is not a functionality that was able to work with the `asyncio` event loop.

### `hyperion_demo`
This is a demo publisher that'll publish signals based on an FBG needle spec. Required to input number of channels and number of active areas
#### Parameters
Same as `hyperion_talker` with the addition
* demo.num_active_areas: the number of active areas for the simulated FBG-sensorized needle
* demo.num_channels: the number of channels for the simulated FBG-sensorized needle

#### Publishers
Same as `hyperion_talker`.

#### Subscibers
Same as `hyperion_talker`.

#### Services
Same as `hyperion_talker`.

### `reconnect`
Reconnects the interrogator by trigginering the interrogators /interrogator/reconnect service. Trigger the reconnect by

``ros2 run hyperion_interrogator reconnect --ros-args -r __ns:=/needle``

### `calibrate_sensors`
Triggers the interrogator to calibrate the sensors by collecting the parameter's number of samples. Trigger the calibraiton by

``ros2 run hyperion_interrogator calibrate_sensors --ros-args -r __ns:=/needle``

## Launch
### Demo Node
To launch the demo node you should use the command

``ros2 launch hyperion_interrogator hyperion_demo.launch.py ip:=<demo IP address of the interrogator> numCH:=<number of FBG channels> numAA:=<number of FBG active areas per channel>``

This launches tyhe hyperion demo node under the namespace `/needle`

### Default Interrogator Node
This launches the hyperion interrogator under the namespace `/needle`

``ros2 launch hyperion_interrogator hyperion_talker.launch.py ip:=<IP address of hyperion interrogator>``

### Peak Streaming Node
This launches the hyperion interrogator under the namespace `/needle`

``ros2 launch hyperion_interrogator hyperion_talker.launch.py ip:=<IP address of hyperion interrogator>``
