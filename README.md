# Complete Copter Drone's Missionn

### 1. Drone's Mission
The mission begins by launching the drone from a predetermined starting position. The drone is then directed to navigate to a specific destination. Once it reaches the destination, the drone descends to a specified altitude. At this point, the computer vision algorithm is activated.

The algorithm identifies one of the nine ArUco markers, and the drone is instructed to land on it. After landing, the drone performs a designated function, such as waiting for a predetermined duration (e.g., 5 seconds), as demonstrated in the example.

Following the execution of the function, the drone returns to its home position. It descends to the designated altitude and lands on the ArUco marker located at the home position.

The complete mission is as follows:

1. Start from a specific position.
2. Go to a specific location.
3. Descend to a specific altitude.
4. Run computer vision algorithm.
5. Land on one of nine ArUco markers.
6. Execute a function or action.
7. Take off and return to the home location.
8. Descend to a specific altitude.
9. Land on the ArUco marker located at the home position.

### 2. Installation:
1. Install ardupilot SITL [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) .
2. Download and build [ardupilot_gazebo](https://github.com/hasanosman601/ardupilot_gazebo) package.
3. Download and build [mavros](https://ardupilot.org/dev/docs/ros-install.html) package.
4. Download and build this package.

### 3. Running:
Now you are ready to run the project:
##### For just two aruco markers
1. > roscore
2. > rosrun gazebo_ros gazebo --verbose iris_arducopter_runway_2_aruco.world
3. go to ardupilot/ArduCopter folder
4. > sim_vehicle.py -f gazebo-iris --map --console
5. > roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555" or roslaunch mavros apm.launch fcu_url:=tcp://<REMOTE_HOST>:<TCP_PORT>@
6. > rosrun copter_project track_aruco.py 
7. > rosrun copter_project mission.py _target_aruco:=25

##### For 10 aruco markers
1. > roscore
2. > rosrun gazebo_ros gazebo --verbose iris_arducopter_runway_10_aruco.world
3. go to ardupilot/ArduCopter folder
4. > sim_vehicle.py -f gazebo-iris --map --console
5. > roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"
6. > rosrun copter_project track_aruco.py 
7. > rosrun copter_project mission.py _target_aruco:=(a number from range 1 to 9)

##### To do the mission on all 10 aruco markers:
you can do mission on all 10 aruco markers with run:
    > mission_all_markers.py



