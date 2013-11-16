%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file createVideo.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Given the edge definition of a tree, creates a video. The input
% should be a nx6 matrix where each row has [x1,y1,z1,x2,y2,z2] for the 2
% 3D points to be visualized. The second input is the output videos
% filename.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = createVideo(edges, filename, k)

  % Visualize each edge addition and save it as an image
  h = figure(1);
  set(gcf, 'Position', get(0,'Screensize')); 
  counter = 1;
  for i = 1 : size(edges,1)

    % Remove the red drawing and draw blue again
    if(i > 1)
      delete(oldh);
      plot3(edges(i-1,[1,4]), edges(i-1,[2,5]), edges(i-1,[3,6]), 'o-'); hold on;
    end
    
    % Draw the latest edge in red
    oldh = plot3(edges(i,[1,4]), edges(i,[2,5]), edges(i,[3,6]), 'ro-'); hold on;
    
    % Save the figure as an image while changing the view
    if(mod(i,k) == 0)
      name = sprintf('im%05d',counter);
      view(counter*3, 28);
      axis([-3.14 3.14 -3.14 3.14 -3.14 3.14]);
      axis square;    
      print(h,name,'-dpng');
      counter = counter+1;
    end
  end
  
  % Create a video out of the images
  system(['ffmpeg -i im%05d.png -y -r 180 -vb 15M -vcodec libx264 ', filename, '.mp4']);
  system('rm -rf im*');
end