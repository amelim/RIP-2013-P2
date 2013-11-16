% @file armJacobian.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Returns the 3x3 Jacobian for a 3dof planar robot
% 
% For example: armJacobian([pi/4,0,0], [2,2,1])
%       q  ... joint angles
%       ql ... link lengths

function J = armJacobian (q, ql)
    % Compute the sin and cos of the angles
    s1 = sin(q(1)); 
    c1 = cos(q(1));
    a12 = q(1) + q(2);
	s12 = sin(a12); 
    c12 = cos(a12);
    a123= q(1) + q(2) + q(3);
    s123 = sin(a123); 
    c123 = cos(a123);
  
    % Set the jacobian
	J = [-ql(1)*s1 - ql(2)*s12 - ql(3)*s123,  -ql(2)*s12 - ql(3)*s123, -ql(3)*s123; 
        ql(1)*c1 + ql(2)*c12 + ql(3)*s123,   ql(2)*c12 + ql(3)*c123,  ql(3)*c123;
        1 1 1];
end