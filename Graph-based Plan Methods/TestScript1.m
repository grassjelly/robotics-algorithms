map = false(10); %Input Map Parameters
map (1:5, 6) = true; %Obstacle Declaration
start_coords = [6, 2]; %Starting Coordinates
dest_coords  = [8, 9]; %Destination Coordinates
drawMapEveryTime = false; %Display Outputs
[route, numExpanded] = DijkstraGrid(map,start_coords,dest_coords,drawMapEveryTime) %Implementation