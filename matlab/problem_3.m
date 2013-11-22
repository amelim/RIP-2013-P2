clear;
clc;
close all;

addpath('./');
addpath('./kinematics');
addpath('./visualization');

% Set RRT parameters and select variant of RRT planner
rrt_variant     = 'goal_directed';
numberOfSamples = 8000;
stepSize        = 0.05;
goalProbability = 0.1;
errorThreshold  = 0.3;
epsilon = 0.0001;
steps = 3000;

% Set initial configuration q_init (in joint space)
ql = [2 2 1];

% Set initial and goal configuration
q       = [1.5707; -1.2708; 0];
q_init  = q;
q_goal  = [1.5707; 1.2708; 0];

% Load scene data
%file = 'file2.scene';
%obst = load(file);     % data in format [tx, ty, theta, dim_x, dim_y]

% Initialize tree, store as array of size 3 x n
G = [q];
E = [1];

qSum = [];
X = [];

% Start timing
tic

while(size(G,2) < numberOfSamples)
    % Sample random configuration

    % move towards goal with probability goalProbability
    if(rand() < goalProbability)
        q_rand = q_goal;
    else
        q_rand = [rand(); rand(); rand()] * 2 * pi- pi;
    end

    % Find nearest point in already grown tree
    index =  getNearest(G', q_rand);
    q_near = G(:, index);
        
    % Move from q_near towards q_rand
    dq1 = q_rand(1) - q_near(1);
    dq2 = q_rand(2) - q_near(2);
    dq3 = q_rand(3) - q_near(3);
    
    dq = [dq1; dq2; dq3];
    
    q_step = q_near + stepSize * (dq/norm(dq));
    
    % Compute workspace position of the candidate
    x = fk(q_step, ql)';
    
    % Compute the error given the following constraint
    dy      = norm(2.8 - x(2));
    dy_prev = inf;
    
    % If there is non-zero error, drive q_near closer to the constraint
    % Utilize forward retraction approach
    q_new = q_step;

    while(norm(dy) > epsilon)
%       if(norm(q_step - q_new) > norm(q_new - q_near))
% %         disp('break')
%         q_step = q_new;
%         break
%       end
      % Recompute end effector position
      x = fk(q_step, ql)';

      % Recompute current error
      dx      = 0;
      dy      = norm(2.8 - x(2));
      dtheta  = 0;
      dError  = [dx; dy; dtheta];

      % Recompute velocities
      vx = dx;
      vy = dy;
      w  = dtheta;
      xDot    = [vx; vy; w];
      xDot = stepSize * xDot;

      % Recompute Jacobian
      J = armJacobian(q_step, ql);
      
      % Update joint velocities
      qDot = inv(J) * xDot;
      q_step = q_step - qDot;    
    end
    
    % Append q_step to tree --> no collision checks needed, since we use an
    % open environment without obstacles
    G = [G, q_step];
    E(length(E) + 1) = index;       % store index of q_near as parent
    
    if(~mod(length(G), 50))
        disp(['Added node ', num2str(length(G)), ' to tree']);
    end
    
    % Use a workspace error threshold
    if(norm(fk(q_step, ql) - fk(q_goal, ql)) < errorThreshold)
        disp(['Goal found']);
        break;
    end
    
%     % Plot data in 3D space
%     plot3(G(1,:), G(2,:), G(3,:), 'bx');
%     grid on; hold on;
%     
%     % Mark initial configuration
%     plot3(G(1,1), G(2,1), G(3,1), 'ro',...
%         'LineWidth',2,...
%         'MarkerEdgeColor','b',...
%         'MarkerFaceColor','r',...
%         'MarkerSize',12)
%     
%     % Mark goal configuration in case of goal directed RRTs
%     if(strcmp(rrt_variant, 'goal_directed'))
%         plot3(q_goal(1), q_goal(2), q_goal(3), 'go',...
%             'LineWidth',2,...
%             'MarkerEdgeColor','b',...
%             'MarkerFaceColor','g',...
%             'MarkerSize',12)
%     end
%     
%     % Draw random sampled config and taken step
%     h_old = plot3(q_rand(1), q_rand(2), q_rand(3), 'bo',...
%         'LineWidth',2,...
%         'MarkerEdgeColor','b',...
%         'MarkerFaceColor','b',...
%         'MarkerSize',12); hold on;
%     
%     % Plot edges of tree
%     Xedge = [];
%     Yedge = [];
%     Zedge = [];
%     for i = 1:1:length(E)
%         Xedge = [G(1, i), G(1, E(i))];
%         Yedge = [G(2, i), G(2, E(i))];
%         Zedge = [G(3, i), G(3, E(i))];
%         line(Xedge, Yedge, Zedge);
%     end
%     
%     % Add labels to axes
%     xlabel('Joint angle 1 q_1','fontsize',14,'fontweight','b')
%     ylabel('Joint angle 2 q_2','fontsize',14,'fontweight','b')
%     zlabel('Joint angle 3 q_3','fontsize',14,'fontweight','b')
%     axis([-pi pi -pi pi -pi pi]);
%     pause(0.01);
%     
%     delete(h_old);
end

% End timing
toc

%% Draw tree
% Plot data in 3D space
plot3(G(1,:), G(2,:), G(3,:), 'bx');
grid on; hold on;

% Mark initial configuration
plot3(G(1,1), G(2,1), G(3,1), 'ro',...
    'LineWidth',2,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor','r',...
    'MarkerSize',12)

% Mark goal configuration in case of goal directed RRTs
if(strcmp(rrt_variant, 'goal_directed'))
    plot3(q_goal(1), q_goal(2), q_goal(3), 'go',...
        'LineWidth',2,...
        'MarkerEdgeColor','b',...
        'MarkerFaceColor','g',...
        'MarkerSize',12)
end

% Plot edges of tree
% for i = 1:1:length(E)
%     X = [G(1, i), G(1, E(i))];
%     Y = [G(2, i), G(2, E(i))];
%     Z = [G(3, i), G(3, E(i))];
%     line(X, Y, Z);
% end

% Add labels to axes
xlabel('Joint angle 1 q_1','fontsize',14,'fontweight','b')
ylabel('Joint angle 2 q_2','fontsize',14,'fontweight','b')
zlabel('Joint angle 3 q_3','fontsize',14,'fontweight','b')
    
%% Reconstruct path and animate it and/or create video
vidoe = false;
file = ''

if(strcmp(rrt_variant, 'baseline'))
    % Pick random node on tree and move arm back to start position
    %q_goal = G(1:3, randi(length(G)));    
    q_last = G(1:3, end); 
elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
    % Pick last node added to tree, i.e. goal configuration
    q_last = G(1:3, end);
end

qSum = q_last;

% Reconstruct path taken
i = length(G);

while(i > 1)
    i = E(i);
    qSum = [qSum G(1:3, i)];
end

if(strcmp(rrt_variant, 'baseline'))
    drawScene(file, qSum', ql, '', fk(q_init, ql), fk(q_goal, ql));
elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
    drawScene(file, qSum', ql, '', fk(q_init, ql), fk(q_goal, ql));
end

% %% Prepare tree for video - call createVideo(tree, 'filename', step)
% tree = zeros(length(E), 6);
% 
% for i = 1:1:length(E)
%     tree(i, :) = [G(1:3, i)', G(1:3, E(i))'];
% end
% 
% createVideo(tree, 'filename', 2)