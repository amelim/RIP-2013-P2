clear;
clc;
close all;

addpath('./');
addpath('./kinematics');
addpath('./visualization');

% Set RRT parameters and select variant of RRT planner
% rrt_variant     = 'baseline'; 
rrt_variant     = 'goal_directed';
% rrt_variant     = 'goal_connect';
numberOfSamples = 200;
stepSize        = 0.05;
goalProbability = 0.02;
errorThreshold  = 0.1;
epsilon = 0.001;
steps = 800;

% Set initial configuration q_init (in joint space)
ql = [2 2 1];

% Set initial and goal configuration
q       = [1.5707; -1.2708; 0];
q_init  = q;
q_goal  = [1.5707; 1.2708; 0];

% Load scene data
file = 'file2.scene';
%obst = load(file);     % data in format [tx, ty, theta, dim_x, dim_y]

% Initialize tree, store as array of size 3 x n
G = [q];
E = [1];

qSum = [];
X = [];

% Start timing
tic

% Set flag for 2c connect variant of RRT goal-directed
connect = false;

while(length(G) < numberOfSamples)
    % Sample random configuration

    % move towards goal with probability goalProbability
    if(rand() > goalProbability || connect)
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
    
    q_step = q_near + stepSize * [dq1; dq2; dq3];
    
    % Compute initial Jacobian matrix
    J = armJacobian(q_step, ql);
    
    % Compute workspace position of the candidate
    x = fk(q_step, ql)';
    
    % Compute the error given the following constraint
    dy      = norm(3 - x(2));
    dy_prev = inf;
    q_new = [];

    % If there is non-zero error, drive q_near closer to the constraint
    if(dy > 0)
        for i = 1:1:steps
            x = fk(q_step, ql)';            
            
            % Recompute current error
            dx      = 0;
            dy      = norm(3 - x(2));
            dtheta  = 0;
            % Recompute velocities
            vx = dx / steps;
            vy = dy / steps;
            w  = dtheta / steps;
            xDot    = [vx; vy; w];
            xDot = 0.05 * xDot / norm(xDot);
            
            if(dy >= dy_prev)
                break;
            end

            dy_prev = dy;
            
           	if(dy < epsilon)
                break
            end            
            q_new = q_step;
            X = [X, x];

            % Compute joint velocities based on linear end effector velocities
            if(rank(J) < 3)
                disp('J is singular');
            end

            % Recompute Jacobian
            J = armJacobian(q_step, ql);
            qDot = inv(J) * xDot

            % Recompute joint angles with inverse Jacobian keeping in mind that
            % xDot = J * qDot --> qDot = inv(J) * xDot
            q_step = q_step + qDot;
            if(norm(q_step - q_new) > 0.5)
                break;
            end

        end
    end
    
    % Append q_new to tree
    collision = 0;
    if(~collision)
        G = [G, q_new];
        E(length(E) + 1) = index;       % store index of q_near as parent
        
        if(~mod(length(G), 10))
            disp(['Added node ', num2str(length(G)), ' to tree']);
        end
    end
    
    if(norm(q_step - q_goal) < errorThreshold)
        disp(['Goal found']);
        break;
    end
end

% End timing
toc

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
for i = 1:1:length(E)
    X = [G(1, i), G(1, E(i))];
    Y = [G(2, i), G(2, E(i))];
    Z = [G(3, i), G(3, E(i))];
    line(X, Y, Z);
end

% Add labels to axes
xlabel('Joint angle 1 q_1','fontsize',14,'fontweight','b')
ylabel('Joint angle 2 q_2','fontsize',14,'fontweight','b')
zlabel('Joint angle 3 q_3','fontsize',14,'fontweight','b')

%% Reconstruct path and animate it and/or create video
vidoe = false;

if(strcmp(rrt_variant, 'baseline'))
    % Pick random node on tree and move arm back to start position
    %q_goal = G(1:3, randi(length(G)));    
    q_goal = G(1:3, end); 
elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
    % Pick last node added to tree, i.e. goal configuration
    q_goal = G(1:3, end);
end

qSum = q_goal;

% Reconstruct path taken
i = length(G);

while(i > 1)
    i = E(i);
    qSum = [qSum G(1:3, i)];
end

if(strcmp(rrt_variant, 'baseline'))
    drawScene(file, qSum', ql, '', fk(q_init, ql), fk(q_goal, ql));
elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
    drawScene(file, qSum', ql, 'test', fk(q_init, ql), fk(q_goal, ql));
end

% %% Prepare tree for video - call createVideo(tree, 'filename', step)
% tree = zeros(length(E), 6);
% 
% for i = 1:1:length(E)
%     tree(i, :) = [G(1:3, i)', G(1:3, E(i))'];
% end
% 
% createVideo(tree, 'filename', 2)