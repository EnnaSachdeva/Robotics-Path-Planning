function [is_solved, timeSpent, closedList, expandedNodes, numExpandedNodes] = Astar2D_search(map, startNode, goalNode, epsilon, currentTime, runTime)

% 2 lists: Open list and closed list
% Open list: contains the nodes that needs to be checked.
% Closed list: that have been visitited
% Heurstic: h(n) = manhattan distance from that tile to goal (no of tiles)
% or the diagonal motion as well
% Cost to go: g(n) = for diagonal- (euclidean distance)
% f(n) = h(n)+g(n)
% select the node (surrounding the parent node) with the least f(n) and put
% the previous (starting node) in the closed list, and node in the closed list and 
% In the beginning, the parent of all nodes surrounding the start node,
% will be the start node itself.
% first we need to make a graph: data structure to create a node in the graph 
% with cost associated with each node in the path.
% Each square in the grid is called a "Node"
% then need to make a queue

expandedNodes = [];
closedList = [];
tic;
timeSpent = 0;
[start_state_x, start_state_y] = state_from_index(map, startNode);
[goal_state_x, goal_state_y] = state_from_index(map, goalNode);

start_state = [start_state_x, start_state_y]; % coordinates
goal_state = [goal_state_x, goal_state_y]; % coordinates


% initialize G cost
G = [];
for i=1:map.C
    for j = 1:map.R
        if (map.cells(i, j) == 0)
            %plot(i, j, '^');
            G_cost(i,j) = inf;%g(n)
        end
    end
end
        

H_cost = calculate_heuristic(map, goal_state, epsilon); % for the entire grid

F_cost = []; % for just open list nodes  (it is the sum of F(n)+G(n)
G_cost(start_state_x, start_state_y) = 0;
F_cost(start_state_x, start_state_y)  = G_cost(start_state_x, start_state_y) + H_cost(start_state_x, start_state_y);

% openList: set of nodes to be evaluated
% closedList: set of nodes already evaluated
% add start node to open


closedList = [];
openList = [start_state_x, start_state_y, G_cost(start_state_x, start_state_y),... 
             F_cost(start_state_x, start_state_y), 0];



is_solved = false;
count = 0;

%tic;
while (~isempty(openList))  % parent pos ~= Target
    %pause(0.001);
     

    
    count = count+1;
    [val,index] = min(openList(:,4)); 
    
    currentState = openList(index,:); %node in openlist with the lowest F_cost
    
    %plot(currentState(1),currentState(2), '*');%'--', 'MarkerSize',10, 'MarkerFaceColor','c');
    %rectangle('Position', [currentState(1)-1/2, currentState(1)-1/2, 1, 1], 'Linewidth', 2, 'FaceColor', 'w', 'EdgeColor', 'w');
    %pause(0);
   
    currentNode = index_from_state(map,currentState(1), currentState(2));
    
    
    % if currentNode is the goal node, path found
     if ( currentNode == goalNode)
         closedList = [closedList;currentState];
        is_solved = true;
        break;
     end
    
    
    %remove current node from open set and add it to closed set
    openList(index,:) = [];
    closedList = [closedList; currentState];
    
    % find neighbours of the currentState/node and for each neighbor, see
    % if the node is traversable or neigbour is in closedlist
    [neighbors,neighborCount]  = get_neighbors(map, currentNode);
    
    %neighbors
    
    
    
    
    % for each neighbor, check if it is in the closedlist or openlist
    for n=1:neighborCount
        
        [neighbor_x, neighbor_y] = state_from_index(map,neighbors(n));
        
        expandedNodes = [expandedNodes; neighbor_x, neighbor_y];
        
       % plot(neighbor_x,neighboSr_y,'--', 'MarkerSize',10, 'MarkerFaceColor','c');
        %rectangle('Position', [neighbor_x-1/2,neighbor_y-1/2, 1, 1], 'Linewidth', 2, 'FaceColor', 'w', 'EdgeColor', 'w');
        %pause(0);
        
        %set(h1,'Visible','off')
       in_closedList = 0;
        for i =1:size(closedList,1)
            if(neighbor_x == closedList(i,1) && neighbor_y==closedList(i,2)) 
                in_closedList = 1;
                % if neighbor is already in the closedlist
                break;
            end
        end
                
         % check if neighbor is in closed list
         if(in_closedList == 1)% in the neighbor is in closed list, check next neighbor
             continue
         end
             
                
         in_openList = 0;   
         % check if neighbor is in open list
        if(~isempty(openList))% if neighbor is in openlist
            for j = 1:size(openList,1)
                if(neighbor_x == openList(j,1) && neighbor_y==openList(j,2))
                     in_openList = j;
                     break;
                end
             end
        end
       
            
         updatedG_cost = G_cost(currentState(1),currentState(2)) + norm([currentState(1)-neighbor_x,currentState(2)-neighbor_y]);
                         
                         
                         
                         
        %%%% if node is not in openlist, add to open list
        if(in_openList==0)
            G_cost(neighbor_x,neighbor_y) = updatedG_cost;
            updatedF_cost =  G_cost(neighbor_x,neighbor_y) + H_cost(neighbor_x,neighbor_y);
            newState = [neighbor_x neighbor_y  G_cost(neighbor_x, neighbor_y) updatedF_cost size(closedList,1)];
            openList = [openList; newState];
            
            %scatter(neighbor_x,neighbor_y,'x','color','green')
            
            continue
        end
        
         %%% if node is in the openlist
         %if updatedG_cost > G_cost, skip
         if (updatedG_cost >= G_cost(neighbor_x,neighbor_y)) % leave that neigbor/node
             continue
         end
               
              
         
         % if updatedG_cost is less than G_cost, then update G_cost
         G_cost(neighbor_x,neighbor_y)= updatedG_cost;
         updatedF_cost(neighbor_x,neighbor_y) = G_cost(neighbor_x,neighbor_y)+H_cost(neighbor_x,neighbor_y);
         openList(in_openList,3:5) = [updatedG_cost updatedF_cost(neighbor_x,neighbor_y) size(closedList,1)];
         
    end     
     
    
   if( currentTime + toc >=  runTime)
        closedList = [];
        expandedNodes = [];
        numExpandedNodes = 0;
                  
       return;
   end
   
             
end 
   timeSpent = timeSpent+toc; 
   
   m = expandedNodes;
   numExpandedNodes =length(unique(m, 'rows'));
    
end

