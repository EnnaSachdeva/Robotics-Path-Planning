% This code performs the RRT path planning in the 2D maze
% Here is the psudo code
% Generate a random configuration x
% check if that configuration is in free space
% find the closest node (y) in the existing tree to this random configuration space
% if Dis(x, y) > delta : check if x is too far from y
% find a configuration z, that is along the path from x to y, such that
% Dist(z, y) <= delta
% x = z;
% if localplanner(x, y)- check if we can go from x to y
% add x to the tree y was its parent
% assumes that we have some distance function

clear all
clc
clf

rng(20); % random seed 

%filename = "../maze1.pgm";
filename = "../maze2.pgm"

map = read_map(filename);

[startNode, num_nodes] = get_start(map);
goalNode = get_goal(map);


[start_state_x, start_state_y] = state_from_index(map, startNode);
[goal_state_x, goal_state_y] = state_from_index(map, goalNode);

start_state = [start_state_x, start_state_y]; % coordinates
goal_state = [goal_state_x, goal_state_y]; % coordinates


plot(start_state_x, start_state_y, 'ko', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(goal_state_x, goal_state_y, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

is_solved = false;


thresh = 1;
currentNode = startNode;
count = 0;
num_iterations = 10000;
periodic = int16(0.1*num_iterations);

% initialize the tree, by adding start node to the tree
nodeCount = 1;
edgeCount = 0;

tree(nodeCount).x = start_state(1);
tree(nodeCount).y = start_state(2);
tree(nodeCount).x_prev = start_state(1);
tree(nodeCount).y_prev = start_state(2);

%plot([tree(nodeCount).x; tree(nodeCount).x_prev],[tree(nodeCount).y; tree(nodeCount).y_prev], 'r');
delta = 1;

tic;
for i=1:num_iterations
    %display(i)
    %pause(0.001);

    
 
    
    % randomly sample a point in the configuration space, with the goal
    % being sampled periodically
    if (rem(i, periodic) == 0)
        x_rand = goal_state_x;
        y_rand = goal_state_y;
    else
        x_rand = map.C*rand();
        y_rand = map.R*rand();
    end
    
    % Collision check of the random samples, if the random point is in the free space or
    % is in the obstacles space
    
    if(check_hit(map, x_rand, y_rand, 0, 0))
        continue;
    end
    
    %%%%% find the nearest point in the existing tree with the closest node
    %%%%% to the random sample
   
    tree_x = cell2mat({tree(:).x}');
    tree_y = cell2mat({tree(:).y}');

    [closestNodeIndex(i), dist] = dsearchn([tree_x, tree_y], [x_rand, y_rand]);
    
    % make this node already present in the tree as the previous node
    x_prev = tree(closestNodeIndex(i)).x;
    y_prev = tree(closestNodeIndex(i)).y;
 
    
    % if distance between nodes > delta, find a configuration along the
    % line joining these 2 nodes, such that its distance from this
    % closest node<= delta;
    % else connect directly
    distance = sqrt((x_rand-x_prev)^2 + (y_rand-y_prev)^2);
    if( distance > delta)
        
            edge_points = [x_prev, y_prev; x_rand, y_rand];
            
            [x_new, y_new] = go_direction(edge_points, delta);
            
            % again check if this new point is inside any obstacle
            if(check_hit(map, x_new, y_new, 0, 0))
                continue; % generate another random node 
            end
            
    else  % connect them directly
            x_new = x_rand;
            y_new = y_rand;
    end
        
    %scatter(x_new, y_new, 45, '*','r','LineWidth',1);
    %plot(x_new, y_new, 'MarkerSize',10, 'MarkerFaceColor','m');

    edge_points = [x_new, y_new; x_prev, y_prev];

    % check if the connected edge is collision free, and add it to tree
    if (check_collision(map, edge_points) == 0)
        nodeCount = nodeCount+1;
        edgeCount = edgeCount +1;
        
        index = closestNodeIndex(i);
        tree(nodeCount).x = x_new;
        tree(nodeCount).y= y_new;
        
        tree(nodeCount).x_prev = x_prev;
        tree(nodeCount).y_prev = y_prev;
        
        tree(nodeCount).index = nodeCount; 
        tree(nodeCount).indexPrev = index;
        
        tree(nodeCount).dist = dist;

              
        edges(edgeCount).x = [x_prev, x_new];
        edges(edgeCount).y = [y_prev, y_new];

        %plot(tree(nodeCount).x, tree(nodeCount).y, '-', 'MarkerSize',2, 'MarkerFaceColor','r');
        plot([tree(nodeCount).x; tree(nodeCount).x_prev],[tree(nodeCount).y; tree(nodeCount).y_prev], 'r');
        %pause(0.01);
    end
    
       
    tree_x = cell2mat({tree(:).x}');
    tree_y = cell2mat({tree(:).y}');

    % check if the goal node is already reached in the tree
    for k=1:length(tree_x)
         if (sqrt((tree(k).x-goal_state_x)^2 + (tree(k).x-goal_state_y)^2) <=thresh)
            %plot([x_new; tree(closestNodeIndex(i)).x],[y_new;  tree(closestNodeIndex(i)).y], 'r');
            is_solved = true;
            break
         end
    end
    
    if is_solved == true
        break;
    end

    
end
        
          

% Generate the path from backwards

if (is_solved)
   disp('Path Found')

    % Generate the path backward
    path(1).x = goal_state_x; 
    path(1).y = goal_state_y;
    
    path(2).x = tree(end).x; % the last node of the tree near the goal
    path(2).y = tree(end).y;
   
    pathIndex = tree(end).indexPrev; 
    
    tree_x = cell2mat({tree(:).x}');
    tree_y = cell2mat({tree(:).y}');
    
    %[~,pathIndex] = ismember([x_prev, y_prev],[tree_x, tree_y],'rows')
    
    j=0;
    
    while 1
        path(j+3).x = tree(pathIndex).x;
        path(j+3).y = tree(pathIndex).y;
        pathIndex = tree(pathIndex).indexPrev;
        if pathIndex == 1
         time = toc;

            break
        end
        j=j+1;
    end
    
    path(end).x = start_state_x; path(end).y = start_state_y;
    
    fprintf('Path Found in Time: %f, Path Length: %d \n ', time, length(path))

    for j = 2:length(path)
       plot([path(j).x; path(j-1).x;], [path(j).y; path(j-1).y], 'c', 'Linewidth', 4);
    %     pause(0);
        end
else
    disp('No path found.');
end
