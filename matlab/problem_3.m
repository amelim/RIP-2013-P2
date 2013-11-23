clear;
clc;
close all;

addpath('./');
addpath('./kinematics');
addpath('./visualization');

% Set RRT parameters and select variant of RRT planner
rrt_variant     = 'goal_directed';
numberOfSamples = 8000;
stepSize        = 0.03;
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

% Mark initial configuration
plot3(G(1,1), G(2,1), G(3,1), 'ro', 'LineWidth',2, 'MarkerEdgeColor','b',...
    'MarkerFaceColor','r','MarkerSize',12);
grid on; hold on;

% Mark goal configuration in case of goal directed RRTs
plot3(q_goal(1), q_goal(2), q_goal(3), 'go', 'LineWidth',2, 'MarkerEdgeColor','b',...
    'MarkerFaceColor','g', 'MarkerSize',12)

% Add labels to axes
xlabel('Joint angle 1 q_1','fontsize',14,'fontweight','b')
ylabel('Joint angle 2 q_2','fontsize',14,'fontweight','b')
zlabel('Joint angle 3 q_3','fontsize',14,'fontweight','b')
axis([-pi pi -pi pi -pi pi]);

while(size(G,2) < numberOfSamples)
    % Sample random configuration

    % move towards goal with probability goalProbability
    if(rand() < goalProbability)
        q_rand = q_goal;
    else
        q_rand = [rand(); rand(); rand()] * 2 * pi - pi;
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
    dy      = norm(2.8868 - x(2));
    dy_prev = inf;
    
    % If there is non-zero error, drive q_near closer to the constraint
    % Utilize forward retraction approach
    q_new = q_step;
    
    i = 0;
    while(norm(dy) > epsilon)
      i = i+1;
      if i > steps
        break
      end
      
      % Close to singularities, the pseudo-inverse may become
      % unstable. To resolve this, we discard samples when the
      % magnitude of adjustment exceeds the original displacement.
      if(norm(q_step - q_new) > norm(q_new - q_near))
        q_step = q_new;
        break;
      end
      
      % Recompute end effector position
      x = fk(q_step, ql)';

      % Recompute current error
      dx      = 0;
      dy      = norm(2.8868 - x(2));
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
      q_step = q_step - stepSize* qDot;    
    end
    
    % Append q_step to tree --> no collision checks needed, since we use an
    % open environment without obstacles
    % Only add if dy is not too big
    if(dy < 2*errorThreshold)
        G = [G, q_step];
        E(length(E) + 1) = index;       % store index of q_near as parent
    end
    
    if(~mod(length(G), 10))
        disp(['Added node ', num2str(length(G)), ' to tree']);
    end
    
    % Use a workspace error threshold
    if(norm(fk(q_step, ql) - fk(q_goal, ql)) < errorThreshold)
        disp(['Goal found']);
        break;
    end
    
    % Plot new data in 3D space
    plot3(G(1,end), G(2,end), G(3,end), 'bx');
    
    % Draw random sampled config and taken step
%     h_old = plot3(q_rand(1), q_rand(2), q_rand(3), 'bo',...
%         'LineWidth',2,...
%         'MarkerEdgeColor','b',...
%         'MarkerFaceColor','b',...
%         'MarkerSize',12); hold on;
    
    % Plot edges of tree
% 	Xedge = [G(1, end), G(1, E(end))];
%     Yedge = [G(2, end), G(2, E(end))];
%     Zedge = [G(3, end), G(3, E(end))];
%     line(Xedge, Yedge, Zedge);
    
    pause(0.01);
%     delete(h_old);
end

% End timing
toc

%% Reconstruct path and animate it and/or create video
vidoe = false;
file = '';

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
    
%% Prepare tree for video - call createVideo(tree, 'filename', step)
tree = zeros(length(E), 6);

for i = 1:1:length(E)
    tree(i, :) = [G(1:3, i)', G(1:3, E(i))'];
end

createVideo(tree, 'filename', 2)