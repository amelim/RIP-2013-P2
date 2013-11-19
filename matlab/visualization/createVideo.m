%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file createVideo.m
% @author Can Erdogan
% @date Nov 09, 2013
% @brief Given the edge definition of a tree, creates a video. The input
% should be a nx6 matrix where each row has [x1,y1,z1,x2,y2,z2] for the 2
% 3D points to be visualized. The second input is the output videos
% filename.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = createVideo(edges, filename, k, edges2)
    drawGrid = false;
    
    % Visualize each edge addition and save it as an image
    h = figure(1);
    if(drawGrid) grid on; end
    set(gcf, 'Position', get(0,'Screensize'));
    counter = 1;
    
    % Check if second tree was provided
    if(nargin < 4)
        edges2 = [];
    end
    
    for i = 1 : max(size(edges,1), size(edges2,1))
        % Remove the red drawing and draw blue again
        if(i > 1)
            if(i < size(edges,1))
                delete(oldh1);
                plot3(edges(i-1,[1,4]), edges(i-1,[2,5]), edges(i-1,[3,6]), 'o-'); hold on;
            end
            
            if(i < size(edges2,1))
                delete(oldh2);
                plot3(edges2(i-1,[1,4]), edges2(i-1,[2,5]), edges2(i-1,[3,6]), 'go-'); hold on;
            end
            if(drawGrid) grid on; end
        end
        
        % Draw the latest edge of tree1 and tree2 in red
        if(i < size(edges,1))
            oldh1 = plot3(edges(i,[1,4]), edges(i,[2,5]), edges(i,[3,6]), 'ro-'); hold on;
        end
        
        if(i < size(edges2,1))
            oldh2 = plot3(edges2(i,[1,4]), edges2(i,[2,5]), edges2(i,[3,6]), 'ro-'); hold on;
        end
            
        if(drawGrid) grid on; end
        
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
    %system(['ffmpeg -i im%05d.png -y -r 180 -vb 15M -vcodec libx264 ', filename, '.mp4']);
    %system('rm -rf im*');
end