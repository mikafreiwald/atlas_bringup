
```
ros2 launch atlas_bringup sim.launch.py 
rviz2 -d src/atlas_bringup/config/atlas.rviz
ros2 launch atlas_bringup navigation_launch.py use_sim_time:=True
```