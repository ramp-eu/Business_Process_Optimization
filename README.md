# Business Process Optimization (BPO)
OPIL module: Business Process Optimization (BPO) Robotics, Control and Decision Systems (RCDS) Laboratory of Cyprus University of Technology (CUT), written and equivalently contintributed by Anatoli A. Tziola, George Georgiades, Savvas G. Loizou from February 2019.

## How to run the BPO module
The guide is tested with docker version 18.09.6 and docker-compose version 1.24.0. Supported $ROS_DISTRO is kinetic or noetic.


## Install from source code
Clone the BPO repository to your catkin workspace. Clone [Business Process Optimization](https://github.com/ramp-eu/Business_Process_Optimization.git) repository into your `catkin_ws` directory. Then compile it with `catkin_make` in one folder up.
```
cp opil_bpo <your_catkin_workspace>/src/
git clone https://github.com/ramp-eu/Business_Process_Optimization.git
cd ..
catkin_make
```

### Starting from source code
To start the BPO service, run the following commands on a computer where ROS kinetic is already installed.

Open a terminal and launch the bpo_service_v5.launch file by running the following command
```
roslaunch opil_bpo bpo_service_v5.launch 
```