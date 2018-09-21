function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = start_coords;
next_point = start_coords;

for i = 1 : max_its
    dist_to_goal = sqrt((next_point(1) - end_coords(1)) ^2 + (next_point(2) - end_coords(2)) ^2);
    if dist_to_goal < 2.0
        return;
    end
    
    v = [gx(round(next_point(2)), round(next_point(1))), gy(round(next_point(2)), round(next_point(1)))];
    v_magnitude = sqrt(sum(v.^2));
    v_norm = v / v_magnitude
    next_point = next_point + [v_norm(1) v_norm(2)];
    route(i+1,:) = [route; next_point];
end
% *******************************************************************
end
