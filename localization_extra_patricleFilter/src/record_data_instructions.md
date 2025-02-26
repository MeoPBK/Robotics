***NOTE: The below instructions are NOT applicable to WS20/21 because it is online. But in case you were to record data to be run on the Create robots, you would follow the below instructions.***

All of the below commands need to be opened in separate terminals. Also make sure sources are built, in the workspace (`catkin_make`)

Start ROS master
```
roscore
```

Start lasers
Make sure the lasers are plugged in to the netbook (silver cables)
```
roslaunch rbo_create laser.launch
```

Start driver
```
rosrun rbo_create driver.py
```
The base will make a few beeps. The base is ready to go only after it plays an "I'm ready" song

Publish static transforms
```
roslaunch rbo_create publish_static_tfs.launch
```

Launch teleoperation node
You are expected to use a keyboard plugged into the USB hub on the robot. You can also accomplish teleoperation with another PC running ROS. But this is involved. Ask supervisors for details
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
The controls are printed on the command line. It basically allows you to use 9 keys to go forward/backward, spin, go in a curved path and halt. Remember, using the default speeds might be too fast. You can change that setting too.

Record bagfile
Change to a directory with reasonable disk space and use the following commands. If the netbook does not have much space left, you can use a thumb drive via the USB hub
```
# To record all topics
rosbag record -a

# To record only the required topics e.g., for making small files
rosbag record /cmd_vel /odom /scan /sensorPacket /tf
```

Now you can use the recorded bagfile for mapping.

