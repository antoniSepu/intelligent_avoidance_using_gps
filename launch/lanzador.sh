#!/bin/bash

#gnome-terminal  --full-screen  \
gnome-terminal  \
  --tab --title "Camera"	--command "bash -c \"
roslaunch pointcloud_to_laserscan rover_camera_wide_scan.launch;
            exec bash\""  \
	--tab --title "Mavros"	--command "bash -c \"
roslaunch mavros apm.launch;
						exec bash\""  \
  --tab --title "Roboclaw"	--command "bash -c \"
roslaunch roboclaw_node roboclaw.launch;
          	exec bash\""  \
  --tab --title "ekf and map"	--command "bash -c \"
roslaunch intelligent_avoidance project_launcher.launch;
          	exec bash\""  \
  --tab --title "RVIZ" --command "bash -c \"
rosrun rviz rviz -d `rospack find intelligent_avoidance`/config/rviz_config.rviz;
						exec bash\""  \
	--tab --title "Avoider"	--command "bash -c \"
rosrun intelligent_avoidance final1.py;
						exec bash\""  \ &
