function [x, y] = go_direction(edge_points, delta)

x_sampled = edge_points(1,1);
y_sampled = edge_points(1,2);

x_tree = edge_points(2,1);
y_tree = edge_points(2,2);


distance = sqrt((x_sampled-x_tree)^2 + (y_sampled-y_tree)^2);

d1 = delta;
d2 = distance- delta;

x = (d1*x_tree + d2*x_sampled)/distance;
y = (d1*y_tree + d2*y_sampled)/distance;

end

