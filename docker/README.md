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


### Starting from docker container
Launch the BPO service from:
 * Docker image

To start the BPO service, pull the BPO docker image from [docker.ramp.eu](https://docker.ramp.eu/) 
and run the docker image on your local computer.

```
docker pull docker.ramp.eu/cut-pvt/mod.ro.bpo:1.0.0
docker run -it docker.ramp.eu/cut-pvt/mod.ro.bpo:1.0.0 bash
```

Otherwise, you can build the docker image locally using from source code using the following commands:
```
cd <your_catkin_workspace>/src/Business_Process_Optimization/docker

docker build -t mod.ro.bpo .
```
or 
* Docker-compose \
Configure the IP addresses and the netinterface of the `docker-compose_bpo.yaml` file regarding your set-up. \
In the `FIWAREHOST`, put the IP address where the fiware is running. \
The `HOST` IP of the computer where BPO service is currently running (if the BPO service is running locally, keep the `127.0.0.1` IP address). \
The `NETINTERFACE` of the computer where the BPO is running. \
Then, run the BPO docker image using the `docker-compose_bpo.yaml` file.
```
docker pull docker.ramp.eu/cut-pvt/mod.ro.bpo:1.0.0

docker-compose -f docker-compose_bpo.yaml up -d
```