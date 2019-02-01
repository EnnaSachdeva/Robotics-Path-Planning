function is_collision = check_collision(map, make_edge)

startx = make_edge(1,1);
starty = make_edge(1,2);
endx = make_edge(2,1);
endy = make_edge(2,2);

% draw the straight line between 2 nodes
distance = sqrt(abs(startx-endx)^2 + abs(starty-endy)^2);

is_collision = 0;

delta = 0.001;
step = 0;
x = 0;

increments = distance/delta;

for i=1:increments
    
    d1 = step*delta;
    d2 = distance- d1;
    
     x = (d1*startx + d2*endx)/(d1+d2);
     y = (d1*starty + d2*endy)/(d1+d2);

    if (check_hit(map, x, y, 0, 0))
        is_collision = 1;
        break;
    else
        step = step+1;
    end
    

end

