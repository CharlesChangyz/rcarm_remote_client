# RC Arm Remote Client

This is a standalone ROS 2 workspace for controlling the rc_arm_2 host GUI over a local network.

## Build

```bash
source /opt/ros/humble/setup.bash
cd "/home/dust/NewDisk1/remote client"
colcon build --symlink-install
```

## Run

```bash
"/home/dust/NewDisk1/remote client/run_remote_client_domain55.sh"
```

Both hosts must be on the same LAN, use `ROS_DOMAIN_ID=55`, and allow ROS 2 DDS discovery traffic through the firewall. This client does not add an application-level safety lock; any trusted machine in the same ROS 2 domain can send the exposed control requests.
