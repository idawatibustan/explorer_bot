# explorer_bot
A prototype search and rescue robot using the Turtlebot in a simulated world.

## How To
1. Compile the simulation script
   ```bash
   chmod +x project_init.sh # or `project_init_2.sh` to load world1 or world2 respectively.
   ```
2. Run simulation
   ```
   ./project_init.sh
   ```
3. Launch the explorer launch file
   ```
   roslaunch explorer_bot explorer_map.launch
   ```
Warning! You may need to `catkin_make` twice. First time tend to fail due to rosservice msg created.

## Package Overview
Nodes <br/>
`wall_detector` processed /scan to detect wall’s ahead, and next block’s left, front and right walls.<br/>
&emsp;`/wall_scan` topic published for walls detected.<br>

`box_navigator` simple navigation to allow the robot move from one box to another. <br/>
&emsp;`/move_north` ros service to move to the north direction.<br/>
&emsp;`/move_east`                             east direction.<br/>
&emsp;`/move_south`                            south direction.<br/>
&emsp;`/move_west`                             west direction.<br/>
&emsp;`/turn_north` ros service to turn to the north direction.<br/>
&emsp;`/turn_east`                             east direction.<br/>
&emsp;`/turn_south`                            south direction.<br/>
&emsp;`/turn_west`                             west direction.<br/>
&emsp;`/is_moving` topic published to indicate robot has movement in progress.<br>

`explorer` path planning and search node that uses wall_detector and box_navigator. <br/>			

Srv <br/>
`MoveGoal` to send `geometry_msgs/Pose2D` as goal to box_navigator.
