function route = DijkstraTorus (input_map, start_coords, dest_coords)
    % Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

%REFERENCE: https://www.coursera.org/learn/robotics-motion-planning/discussions/weeks/2/threads/mWMCtkPjEeiORw7rSWiWog

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);

label = true;

input_map(:, 181) = [];
input_map(181, :) = [];

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    %image(1.5, 1.5, map);
    %grid on;
    %axis image;
    %drawnow;
%     
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distances(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    neighbors = [];
    total_cells = nrows * ncols;
    
    %THIS BLOCK APPENDS neighbors ARRAY for all the neighbors of the current node
    
    %check if the current node is on the left limit of the matrix
    if (j == 1)
      neighbors = [neighbors, current + nrows, total_cells - (nrows-current)];
    %check if the current node is on the right limit of the matrix
    elseif(j == nrows)
      neighbors = [neighbors, current - nrows, nrows - (total_cells - current)];
    %otherwise it's safe to append both left and right neighbor
    else 
      neighbors = [neighbors, (current - nrows), (current + nrows)];
    end;
     
    %check if the current node is on the upper limit of the matrix
    if(i == 1)
      neighbors = [neighbors, current + 1, current + (nrows-1)];
    %check if the current node is on the lower limit of the matrix
    elseif(i == ncols)
      neighbors = [neighbors, current - 1, current - (nrows-1)];
    %otherwise it's safe to append both upper and lower neighbor
    else
      neighbors = [neighbors, (current - 1), (current + 1)];
    end;
    
    for i = 1 : numel(neighbors) %ALGO7
      index = neighbors(i);
      element = (map(index));
      %make sure the current neighbor doesn't hit the obstacle
      if(element == 1 ||  element == 4 || element == 6 && element ~= 2)
        if(distances(index) > min_dist + 1) %ALGO 8
          %save the edge's distance if it's better than previous one
          distances(index) = min_dist + 1; %ALGO 9
          parent(index) = current; %ALGO 10
          %set current neighbor as visited
          map(index) = 4; %ALGO 11
        end;
      end;
    end;
    % *******************************************************************
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    drawMap(label);
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

    function drawMap(label)
        if label==true
        for k = 2:length(route) - 1        
            map(route(k)) = 7;
        end
        image(1.5, 1.5, map);
        grid on;
        axis image;
        end
        end
end
