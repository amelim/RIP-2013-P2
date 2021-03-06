%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file drawArm3.m
% @author Can Erdogan
% @date Jan 21, 2012
% @brief Displays the 3D arm motion given the link lengths, the joint angles trajectory and the 
% time step between each joint angle display. The arm has 3 rotary joints all in the same plane.
% @example drawArm3([0:0.01:1;0:0.02:2;0:0.01:1]',[2,2,1],0.1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = drawArm3 (qs, ql, dt, video, filename, ps, pg) 
    % For each entry in the trajectory, determine the joint locations 
    counter = 1;
    k = 1;             % safe every 5th frame
    
	for index = size(qs, 1) : -1 : 1
%     for index = 1:1:size(qs, 1)
        if(size(qs,1) > 1), clf; end;

        if(nargin > 3 && ~isempty(filename) && isstr(filename))
            data = load(filename);

            for i = 1 : size(data,1)
                drawRectangle(data(i, 1:3), data(i, 4:5));
            end
        end

        % Compute the joint1 location
        j1x = cos(qs(index,1)) * ql(1);
        j1y = sin(qs(index,1)) * ql(1);

        % Compute the joint2 location
        j2x = j1x + cos(qs(index,1) + qs(index,2)) * ql(2);
        j2y = j1y + sin(qs(index,1) + qs(index,2)) * ql(2);

        % Compute the end-effector location
        eex = j2x + cos(sum(qs(index,:))) * ql(3);
        eey = j2y + sin(sum(qs(index,:))) * ql(3);

        % Plot the two lines
        plot([0.0;j1x], [0.0;j1y], 'o-'); hold on;
        plot([j1x;j2x], [j1y;j2y], 'go-'); hold on;
        plot([j2x;eex], [j2y;eey], 'co-'); hold on;
        axis([-10, 10, -10, 10]); hold on;

        % Plot start and target position
        if(nargin > 4)
            plot(ps(1), ps(2), 'bx');
            plot(pg(1), pg(2), 'rx');
        end
        axis equal

        % Wait for update
        if(video)
            if(mod(counter, k) == 0)
                name = sprintf('im%05d',counter);
                print(gcf,name,'-dpng');
            end
            
            counter = counter + 1;
        end
        
        pause(dt);
    end
end
