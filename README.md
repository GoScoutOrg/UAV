# UAV SITL Setup

Thankfully there is a lot of documentation on how to perform this setup. I'll do my best to consolidate it here. This will only urn on Ubuntu.
I'm using 20.04, Gazebo will not run on anything newer so I would stick to that version.

## Dependencies
  
  ### [Ros noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) - The gazebo sim runs in a ros node and this helps make message transportation easier
  
  ### Gazebo - This does the actual rendering of our simulated UAV
  
  ### Ardupilot - You'll have to clone and build the repo for this simulated autopilot
  
  ### MavProxy - This is some wack gcs that the SITL uses, it looks bad but its a need to have, install with `pip install mavproxy`
  
  
 ## Installation
 
  Just follow the instrucitons in these links in the order they are listed
  
  - [MavRos/catkin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)
  - [Ardupilot/MavProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)
  - [Gazebo Ardupilot plugins](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)
  
 ## Running the SITL
 
 There are 3 processes to get everyhting working. The gazebo sim, the SITL, the script sending commands to Ardupilot. First get the gazebo sim running by opening a terminal and inputing
 ``roslaunch iq_sim runway.launch``
 
Open another terminal and start the sitl
``python ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console``

Lastly just run the script you've been developing to program the autopilot. To connect just make sure you are specifying local host on port 14550

## More things:
To connect to the drone:
on Linux run the following:
  - sudo create_ap --freq-band 2.4 [interface] CLAUSWORKS-PC clausworks
  - sudo firwall-cmd --zone=nm-shared --add-interface=ap0
  - sudo arp-scan #To check
