#!/bin/bash

#gnome-terminal  --full-screen  \
gnome-terminal  \
  --tab --title "Camera"	--command "bash -c \"
roslaunch pointcloud_to_laserscan rover_camera_wide_scan.launch;
          exec bash\""  \
	--tab --title "Mavros"	--command "bash -c \"
roslaunch mavros apm.launch;
						exec bash\""  \
  --tab --title "fusion_odom"	--command "bash -c \"
rosrun intelligent_avoidance fusion_odoms.py;
            exec bash\""  \
  --tab --title "geonav"	--command "bash -c \"
roslaunch geonav_transform geonav_transform.launch;
            exec bash\""  \
  --tab --title "Tfs y mapa"	--command "bash -c \"
roslaunch intelligent_avoidance tfs.launch;
            exec bash\""  \
  --tab --title "RVIZ" --command "bash -c \"
rosrun rviz rviz -d `rospack find intelligent_avoidance`/config/rviz_config.rviz;
						exec bash\""  \
	--tab --title "Avoider"	--command "bash -c \"
rosrun intelligent_avoidance final1.py;
						exec bash\""  \ &
