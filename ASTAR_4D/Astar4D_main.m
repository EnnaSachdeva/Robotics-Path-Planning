% This code performs the A* algorithm in the 4D maze

clear all
clc
close all
clf


epsilon = 10;

%%%%%%%%%%%%% Uncomment the below 2 lines of code %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% if the epsilon value has to  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% be user defined  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% prompt = 'Enter epsilon value between 1 and 10? ';
% epsilon = input(prompt)



%filename = "../maze1.pgm";
filename = "../maze2.pgm"
map = read_map_for_dynamics(filename);

[startNode, num_nodes] = get_start_dynamic(map);
goalNode = get_goal_dynamic(map);

% dividing into grids
runtime = 1;


time = 0;

count = 0;

closedNodes = [];
expandedNodes = [];
while (time<runtime)
    
   
        count = count +1;
         tic;
   
    
    
         [is_solved, timeSpent, closedList, expandedList, numExpandedNodes] = Astar4D_search(map, startNode, goalNode, epsilon, time, runtime);
                   
          time = time+toc;
         
         if (~isempty(closedList)) % the time is over
             
              closedNodes = closedList;
              expandedNodes = expandedList;
                 % finding the pathlength
              m = size(closedList,1);
              path = [];
              while(m > 0)

                    x = closedList(m,1);
                    y = closedList(m,2);
                    vx = closedList(m,3);
                    vy = closedList(m,4);
                    m = closedList(m,7);
                    path = [x, y, vx, vy;path];

                end
         else
             
             display('Time Over');
             break;
            
         end
    
    

        if (is_solved)
           fprintf('Path Found, epsilon: %f, Time taken to run this search: %f, Expanded Nodes: %d, Path Length: %d \n ', epsilon, timeSpent, numExpandedNodes, size(path,1))
           %plotPath(closedList, expandednodes);
        else
            fprintf('No Path Found')
        end

        if epsilon == 1.0000
            break;
        end

        epsilon = epsilon - 0.5*(epsilon -1); 
        
        if epsilon <1.001
           epsilon = 1.000;
            
       end
         
        
    
end


    if (time>= runtime)
        fprintf('Time Limit of %f exceeded \n', runtime)
    end
        
    
   
    % plot the path corresponding to the final completed epsilon
    plotPath(closedNodes, expandedNodes);

    

