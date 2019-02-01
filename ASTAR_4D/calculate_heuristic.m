function Heuristic = calculate_heuristic(map,goal, epsilon)
%Calculates heuristic of each node in the map/grid

for i=1:map.C
    for j = 1:map.R 
        if (map.cells(i, j) == 0)
            %plot(i, j, '^');
            Heuristic(i,j) = epsilon*(norm([goal(1), goal(2)]-[i,j])/(map.maxV*sqrt(2))); % euclidean distance
        end
    end
              
end

