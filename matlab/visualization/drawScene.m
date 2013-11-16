% ==================================================================================================
% @file drawScene.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Draws a scene reading the position/orientation (x,y,th) and
% dimensions (w,h) from the given file, and the arm.
% ==================================================================================================

function [] = drawScene (file, q, ql, q_init, q_goal) 
  data = load(file);
  for i = 1 : size(data,1)
    drawRectangle(data(i, 1:3), data(i, 4:5));
  end
  
  hold on;
  if(nargin > 3)
      drawArm3(q, ql, 0.1, file, q_init, q_goal)
  else
      drawArm3(q, ql, 0.1, file)
  end
  set(gcf, 'Position', get(0,'Screensize')); 
  axis equal;
end
