# Robotics-Path-Planning

A star and RRT for 2D and 4D maze environment. The 2D maze is a 2D environment, discrete or
continuous consisting in which the state is defined as (x, y), and
the 4D maze is a 2D environment with 4D state space, consisting
of position and velocities along x and y axes. Therefore, the state
space becomes (x; y; vx; vy). 

For the 2D maze, the A-star algorithm has been implemented with an admissible heuristic as the euclidean distance
of an state from the goal position. Start Position = (1, 1), Goal Position = (25, 25).

For the 4D maze, the state consists of (x; y; vx; vy). Therefore, instead of optimizing the distance in the 2D
state environment, here the admissible heuristic can be the
one which minimizes the time to reach the goal. h(x; y) = sqrt((x − xgoal)^2 + (y − ygoal)^2)/vmax, where 
max = sqrt(vxmax)^2 + (vymax)^2)
