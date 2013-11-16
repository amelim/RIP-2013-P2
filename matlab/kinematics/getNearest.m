%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file getNearest.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Returns the index to the nearest node to 'qRand' by linear search.
% We use this as opposed to knnsearch because somehow this is faster.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function index = getNearest(nodes, qRand)
  minDistSq = 10000000;
  
  %disp(['Number of samples to search: ', num2str(size(nodes,1))]);
  
  for i = 1 : size(nodes,1)
    node = nodes(i,:);
    distSq = (node(1)-qRand(1))*(node(1)-qRand(1))+(node(2)-qRand(2))*(node(2)-qRand(2))+(node(3)-qRand(3))*(node(3)-qRand(3));
    if(distSq < minDistSq)
      index = i;
      minDistSq = distSq;
    end
  end
end