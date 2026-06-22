# RC Arm Remote Client

This is a standalone ROS 2 workspace for controlling the rc_arm_2 host GUI over a local network.

## Build

```bash
source /opt/ros/humble/setup.bash
cd "/home/dust/NewDisk1/remote client"
colcon build --symlink-install
```

## Run

The default launcher uses Fast DDS discovery server because some Wi-Fi/AP
multicast paths do not reliably forward ROS 2 DDS discovery traffic.

Known network for this setup:

- GUI / discovery server computer: `192.168.3.83`
- Remote ROS node computer: `192.168.3.159`
- `ROS_DOMAIN_ID=55`
- ROS 2 Humble
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISCOVERY_SERVER=192.168.3.83:11811`

### 1. Start Fast DDS discovery server on the GUI computer

```bash
cd "/home/dust/NewDisk1/remote client"
./run_fastdds_discovery_server.sh
```

Equivalent manual command:

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l 192.168.3.83 -p 11811
```

### 2. Start remote ROS nodes on `192.168.3.159`

Set the same discovery server before starting nodes:

```bash
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.3.83:11811
source /opt/ros/humble/setup.bash
```

Then launch the ROS node stack normally.

### 3. Start this remote GUI

```bash
"/home/dust/NewDisk1/remote client/run_remote_client_domain55.sh"
```

The launcher exports:

```bash
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DISCOVERY_SERVER=192.168.3.83:11811
```

You can override the defaults without editing the script:

```bash
DISCOVERY_SERVER_HOST=192.168.3.83 \
DISCOVERY_SERVER_PORT=11811 \
"/home/dust/NewDisk1/remote client/run_remote_client_domain55.sh"
```

Both hosts must be on the same LAN and be able to reach UDP/TCP traffic for the
Fast DDS discovery server endpoint. This client does not add an
application-level safety lock; any trusted machine using the same discovery
server/domain can send the exposed control requests.
