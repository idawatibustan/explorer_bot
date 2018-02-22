# explorer_bot
A prototype search and rescue robot using the Turtlebot in a simulated world.

## Package Overview
Nodes <br/>
`wall_detector` processed /scan to detect wall’s ahead, and next block’s left, front and right walls.<br/>
&emsp;`/wall_scan` topic published for walls detected.<br>
`box_navigator` simple navigation to allow the robot move from one box to another. <br/>
&emsp;`/move_x` ros service to allow movement in x direction.<br/>
&emsp;`/move_y` ros service to allow movement in y direction.<br/>
`explorer` path planning and search node that uses wall_detector and box_navigator. <br/>			
