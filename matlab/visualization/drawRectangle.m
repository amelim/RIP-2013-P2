% ==================================================================================================
% @file drawRectangle.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Draws a rectangle with the given dimensions and the transform for (x,y,th).
% ==================================================================================================

function [] = drawRectangle(t, dims)

  % ========================================================================
  % Get the rectangle vertices positions
  
  % Get the vertices of the rectangle in local frame
  rv1_ = [dims(1)/2, dims(2)/2, 1];
  rv2_ = [dims(1)/2, -dims(2)/2, 1];
  rv3_ = [-dims(1)/2, -dims(2)/2, 1];
  rv4_ = [-dims(1)/2, dims(2)/2, 1];
  
  % Create the transform that represents the location and orientation of
  % the rectangle in world frame
  T = [ cos(t(3)), -sin(t(3)), t(1);
        sin(t(3)),  cos(t(3)), t(2);
        0,          0,         1];
      
  % Transform the vertices to world coordinates    
  rv1 = T * rv1_';
  rv2 = T * rv2_';
  rv3 = T * rv3_';
  rv4 = T * rv4_';
 
  % Draw the rectangle
  plot([rv1(1); rv2(1)], [rv1(2); rv2(2)], '-'); hold on;
  plot([rv2(1); rv3(1)], [rv2(2); rv3(2)], '-'); hold on;
  plot([rv3(1); rv4(1)], [rv3(2); rv4(2)], '-'); hold on;
  plot([rv4(1); rv1(1)], [rv4(2); rv1(2)], '-'); hold on;
  axis([-10 10 -10 10]);

end