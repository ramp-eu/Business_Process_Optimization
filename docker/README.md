# mod.sw.bpo
OPIL module: Business Process Optimization (BPO) Robotics, Control and Decision Systems (RCDS) Laboratory of Cyprus University of Technology (CUT), written and equivalently contintributed by Anatoli A. Tziola, George Georgiades, Savvas G. Loizou from February 2019.

## Install from source code
Supported $ROS_DISTRO is kinetic.

Clone the BPO repository to your catkin workspace. Clone `mod.sw.bpo` repository into your `catkin_ws` directory. Then compile it with `catkin_make` in one folder up.
```
cp opil_bpo <your_catkin_workspace>/src/
git clone https://github.com/rcdslabcut/mod.sw.bpo.git
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
To start the BPO service, build the BPO docker container using the `Dockerfile`.
```
cd <your_catkin_workspace>/src/mod.sw.bpo/
docker build -t mod.sw.bpo .
```

Launch the BPO service from:
 * Docker image
```
docker run -it mod.sw.bpo bash
```
or 
 * Docker-compose \
Configure the IP addresses and the netinterface of the `docker-compose_bpo.yaml` file regarding your set-up. \
In the `FIWAREHOST`, put the IP address where the fiware is running. \ 
The `HOST` IP of the computer where BPO service is currently running (if the BPO service is running locally, keep the `127.0.0.1` IP address). \
The `NETINTERFACE` of the computer where the BPO is running. \
Then, run the BPO docker image using the `docker-compose_bpo.yaml` file.
```
docker-compose -f docker-compose_bpo.yaml up -d
```
