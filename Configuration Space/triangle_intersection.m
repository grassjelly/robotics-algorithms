function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

%Reference: https://blogs.mathworks.com/loren/2011/08/29/intersecting-lines/?s_cid=fb_wall_10-21-11_loren_lines

%function prototypes
slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
intercept = @(line,m) line(1,2) - m*line(1,1);
isPointInside = @(xint,myline) (xint >= myline(1,1) && xint <= myline(2,1)) || (xint >= myline(2,1) && xint <= myline(1,1));

for i = 1 : length(P1)
    %assign vertices of line segment 1 where vertices are the current index of triangle and index + 1, 
    if (i < length(P1))
        line1 = [P1(i, 1) P1(i, 2); P1(i + 1, 1) P1(i + 1, 2)];
    else
        line1 = [P1(i, 1) P1(i, 2); P1(1, 1) P1(1, 2)];
    end
 
    for j = 1: length(P2)
        %assign vertices of line segment 2 where vertices are the current index of triangle and index + 1, 
        if (j < length(P2))
         
            line2 = [P2(j, 1) P2(j, 2); P2(j + 1, 1) P2(j + 1, 2)];
        else
            line2 = [P2(j, 1) P2(j, 2); P2(1, 1) P2(1, 2)];
        end
        
        %get the slope of each line
        m1 = slope(line1);
        m2 = slope(line2);
        
        %get the intercept of each line
        b1 = intercept(line1,m1);
        b2 = intercept(line2,m2);
        
        %get the point of intersection
        xintersect = (b2-b1)/(m1-m2)
        yintersect = m1*xintersect + b1
        
        %check if the point of intersection is inside each line
        inside = isPointInside(xintersect,line1) && isPointInside(xintersect,line2);
        
        if(inside == 1)
          flag = true;
          return;
        end;
    end
end

flag = false;
% *******************************************************************
end