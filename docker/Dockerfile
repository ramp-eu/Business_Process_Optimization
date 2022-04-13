FROM osrf/ros:kinetic-desktop-full

RUN apt-get update && apt-get install -y \
	    ros-kinetic-desktop-full \
	    ros-kinetic-navigation \
       	    ros-kinetic-industrial-msgs \
            sudo \	    
            python-requests \			
	    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y

RUN mkdir -p /root/catkin_ws/src
	
RUN cd /root/catkin_ws/ && \
	/bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'

COPY opil_bpo /root/catkin_ws/src/opil_bpo
COPY maptogridmap /root/catkin_ws/src/maptogridmap

RUN cd /root/catkin_ws/src && \
	git clone -b v0.2.0 --recursive https://github.com/iml130/firos.git
	
RUN cd /root/catkin_ws/ && \
	/bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'
	
WORKDIR /root/catkin_ws

COPY docker/firosconfig /root/catkin_ws/src/firos/config

COPY ros_entrypoint.sh /

RUN echo 'source /opt/ros/kinetic/setup.bash \n source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

WORKDIR /

RUN chmod +x /ros_entrypoint.sh
