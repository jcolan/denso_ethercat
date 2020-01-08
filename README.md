# denso_ethercat
Ethercat interface for Denso VS050


## Prerequisites

What things you need to install the software and how to install them

* SOEM
```
sudo apt-get install ros-kinetic-soem
```

* EtherCAT Manager (updated in ROS server until kinetic only)
```
sudo apt-get install ros-kinetic-ethercat-manager
```

* ROS control packages
```
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install -y ros-kinetic-transmission-interface
sudo apt-get install -y ros-kinetic-joint-limits-interface
```

* Provide root privileges
```
sudo setcap cap_net_raw,cap_ipc_lock=+ep ~/catkin_ws/devel/lib/denso_control_ethercat/main
```


## Running the tests

* Joint Position Controller
```
roslaunch denso_ethercat denso_control_pos.launch
```


* Joint Velocity Controller
```
roslaunch denso_control_vel denso_control_pos.launch
```

* Slaves EtherCAT information
```
roslaunch denso_control_vel slaves_info.launch
```
