# explorer_bot
A prototype search and rescue robot using the Turtlebot in a simulated world.

## Package Overview
Nodes			      Details
wall_detector 	processed /scan to detect wall’s ahead, and next block’s left, front and right walls.
	/wall_scan		  topic published for walls detected
box_navigator 	simple navigation to allow the robot move from one box to another
  /move_x 		    ros service to allow movement in x direction
  /move_y 		    ros service to allow movement in y direction
explorer	 	    path planning and search node that uses wall_detector and box_navigator
				
