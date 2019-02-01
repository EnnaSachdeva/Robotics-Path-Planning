% map = read_map_for_dynamics
% By Geoff Hollinger, 2007
%
% Reads in a map for use in the 2D kinematics maze problem.
% Provide filename of binary pgm: 
% i.e map = read_map('maze1.pgm')

function map = read_map(filename)

im = imread(filename);

im(find(im == 0)) = 1;  % denotes the black region as 1 in image; denotes obstacles
im(find(im == 255)) = 0;  % denotes the white region as 0; denotes free space
%im = im';
map.R = size(im,1); % rows in the map
map.C = size(im,2); % columns in the map
map.cells = im; 

%imshow(map.cells);
%figure
imagesc(map.cells); % it displays map.cells as an image
map.cells = map.cells';
hold on
% for i =1:map.C
%     for j = 1:map.R
%          
%          if (map.cells(i,j) == 0)
%              marker = '*';
%          else
%              marker = '^';
%          end
%          plot(i, j, marker, 'MarkerFaceColor','g')
%          
%          pause(0.001)
%     end
% end