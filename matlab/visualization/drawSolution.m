%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file drawSolution.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Given the edge definition of a tree and the indices to the edges that describes the
% path, outputs the path image in the requested view angle. The edge matrix should be a nx6 matrix 
% where each row has [x1,y1,z1,x2,y2,z2] for the 2 3D points to be visualized. The second input is 
% a vector of integers.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = drawSolution (nodes, edges, path, viewAngle, viewArm)

  % Draw the tree
  h = figure(1);
  set(gcf, 'Position', get(0,'Screensize')); 

  for i = 1 : size(edges,1)
    plot3(edges(i,[1,4]), edges(i,[2,5]), edges(i,[3,6]), 'o-'); hold on;
  end
  
  % Draw the edges
  pathNodes = [];
  for i = 1 : numel(path) - 1
    pathNodes(end+1, :) = nodes(path(i),1:3);
    plot3(nodes(path(i:i+1),1), nodes(path(i:i+1),2), nodes(path(i:i+1),3), 'o-g', 'LineWidth', 2); hold on;
  end
  pathNodes(end+1, :) = nodes(path(end),1:3);

  % Play with visualization
  view(viewAngle, 28);
  axis([-3.14 3.14 -3.14 3.14 -3.14 3.14]);
  axis square;    
  
  % Draw the arm motion
  if(viewArm)
    figure;
    set(gcf, 'Position', get(0,'Screensize')); 
		drawArm3(pathNodes,[2 2 1], 0.01);
  end
end