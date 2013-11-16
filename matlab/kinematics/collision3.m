% ==================================================================================================
% @file collision.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Checks for collision between a 3-link planar robot and a
% rectangular object in 2D world. Assume that the robot base is at (0,0)
% @params q: joint angles, t: (x,y,th) for rectangle, ql: link lengths,
%         dims: (w,h) dimensions of the rectangle
% ==================================================================================================

function coll = collision3(q, t, ql, dims)

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
  rv1 = T * rv1_'; rv1 = rv1(1:2);
  rv2 = T * rv2_'; rv2 = rv2(1:2);
  rv3 = T * rv3_'; rv3 = rv3(1:2);
  rv4 = T * rv4_'; rv4 = rv4(1:2);
  redges = {{rv1, rv2}, {rv2, rv3}, {rv3, rv4}, {rv4, rv1}};
  
  % ========================================================================
  % Get the robot link vertices positions
  
  lv1 = [0,0];
  lv2 = ql(1) * [cos(q(1)), sin(q(1))]; 
  lv3 = lv2 + ql(2) * [cos(q(1)+q(2)), sin(q(1)+q(2))];
  lv4 = lv3 + ql(3) * [cos(q(1)+q(2)+q(3)), sin(q(1)+q(2)+q(3))];
  ledges = {{lv1, lv2}, {lv2, lv3}, {lv3, lv4}};
  
  % ========================================================================
  % Check for collisions between each link and each side of the rectangle
  
  for i = 1 : 4
    for j = 1 : 3
      edge1 = redges{i};
      edge2 = ledges{j};
      out = lineSegmentIntersect([edge1{1}', edge1{2}'], [edge2{1}, edge2{2}]);
      coll = (out.intAdjacencyMatrix == 1);
      %keyboard
      if(coll), return; end;
    end
  end
  
end