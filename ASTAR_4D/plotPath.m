function plotPath(closedList, expandednodes)
 %Path plottingplot matlab
    
    m = size(closedList,1);
    n = size(expandednodes,1);
    path = [];
    neighbors = [];
    
    while(m > 0)
        
        x = closedList(m,1);
        y = closedList(m,2);
        vx = closedList(m,3);
        vy = closedList(m,3);
        m = closedList(m,7);
        path = [x,y;path];
       
    end
    
    for i =1:n
        x = expandednodes(i,1);
        y = expandednodes(i,2);
        neighbors = [neighbors; x, y];
    end
    
         %plot(path(:,1), path(:,2),'MarkerSize',50, 'MarkerFaceColor','green');
    for j = 1:n
       
        %plot(path(j,1),path(j,2),'MarkerSize',10, 'MarkerFaceColor','green');
        rectangle('Position', [neighbors(j,1)-1/2, neighbors(j,2)-1/2, 1, 1], 'Linewidth', 2, 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.85 0.85 0.85]);
        
    end
    
    %plot(path(:,1), path(:,2),'MarkerSize',50, 'MarkerFaceColor','green');
    for j = 1:size(path,1)
        %plot(path(j,1),path(j,2),'MarkerSize',10, 'MarkerFaceColor','green');
         
        rectangle('Position', [path(j,1)-1/2, path(j,2)-1/2, 1, 1], 'Linewidth', 2, 'FaceColor', 'm', 'EdgeColor', 'm');
        
    end
    

end

