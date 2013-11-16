% @file fk.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Returns the (x,y,th) of the end-effector for a given joint angles
% and link lengths.
%
% For example: fk([pi/4,0,0], [2,2,1])
%       q  ... joint angles
%       ql ... link lengths

function t = fk(q, ql)

  % Compute the joint1 location
  j1x = cos(q(1)) * ql(1);
  j1y = sin(q(1)) * ql(1);

  % Compute the joint2 location
  j2x = j1x + cos(q(1) + q(2)) * ql(2);
  j2y = j1y + sin(q(1) + q(2)) * ql(2);

  % Compute the end-effector location
  t = [];
  t(1) = j2x + cos(sum(q(:))) * ql(3);
  t(2) = j2y + sin(sum(q(:))) * ql(3);
  t(3) = sum(q(:));
end