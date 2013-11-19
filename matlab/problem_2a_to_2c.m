clear;
clc;

addpath('./');
addpath('./kinematics');
addpath('./visualization');

% Set RRT parameters and select variant of RRT planner
rrt_variant     = 'baseline'; 
% rrt_variant     = 'goal_directed';
% rrt_variant     = 'goal_connect';
numberOfSamples = 300;
stepSize        = 0.05;
goalProbability = 0.02;
errorThreshold  = 0.1;

% Set initial configuration q_init (in joint space)
ql = [2 2 1];
if(strcmp(rrt_variant, 'baseline'))
    % Set initial configuration
    q = [0; 0; 0];
    q_init = q;
    
    % Load scene data
    file = 'file1.scene';
    obst = load(file);     % data in format [tx, ty, theta, dim_x, dim_y]
    dim = [obst(4), obst(5)];
elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
    % Set initial and goal configuration
    q       = [0.14; 1.45; 1.0];
    q_init  = q;
    q_goal  = [-2.5; -0.5; -2.1];
    
    % Load scene data
    file = 'file2.scene';
    obst = load(file);     % data in format [tx, ty, theta, dim_x, dim_y]
else
    
end

% Initialize tree, store as array of size 3 x n
G = [q];
E = [1];

% Start timing
tic

% Set flag for 2c connect variant of RRT goal-directed
connect = false;

while(length(G) < numberOfSamples)
    % Sample random configuration
    if(strcmp(rrt_variant, 'baseline'))
        % sample a point uniformly in the range of -pi to pi 
        % (one value for each joint)
        q_rand = [rand(); rand(); rand()] * 2 * pi- pi; 
    elseif(strcmp(rrt_variant, 'goal_directed'))
        % move towards goal with probability goalProbability
        if(rand() > goalProbability || connect)
            q_rand = q_goal;
        else
            q_rand = [rand(); rand(); rand()] * 2 * pi- pi; 
        end
    elseif(strcmp(rrt_variant, 'goal_connect'))
        % move towards goal with probability goalProbability or if the
        % connect flag is set (i.e. move towards goal as long as you can
        if(rand() > goalProbability || connect)
            q_rand = q_goal;
            
            % Set connect flag so that RRT moves towards goal in the next
            % iteration too
            connect = true;
        else
            q_rand = [rand(); rand(); rand()] * 2 * pi- pi; 
        end
    end

    % Find nearest point in already grown tree
    index =  getNearest(G', q_rand);
    q_near = G(:, index);
    
    % Move from q_near towards q_rand
    dq1 = q_rand(1) - q_near(1);
    dq2 = q_rand(2) - q_near(2);
    dq3 = q_rand(3) - q_near(3);
    
    q_new = q_near + stepSize * [dq1; dq2; dq3];
    
    % Append q_new to tree
    collision = 0;
    if(strcmp(rrt_variant, 'baseline'))
        collision = collision3(q_new, obst(:, 1:3), ql, obst(:, 4:5));
    elseif(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
        for j = 1:1:size(obst,1)
            collision = collision3(q_new, obst(j, 1:3), ql, obst(j, 4:5));
            
            if(collision)
                % If a collision is detected stop moving towards the goal
                connect = false;
                break;
            end
        end
    end
    
    if(~collision)
        G = [G, q_new];
        E(length(E) + 1) = index;       % store index of q_near as parent
        
        if(~mod(length(G), 10))
            disp(['Added node ', num2str(length(G)), ' to tree']);
        end
    end
    
    if(strcmp(rrt_variant, 'goal_directed') || strcmp(rrt_variant, 'goal_connect'))
        if(norm(q_new - q_goal) < errorThreshold)
            disp(['Goal found']);
            break;
        end
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

%% Prepare tree for video - call createVideo(tree, 'filename', step)
tree = zeros(length(E), 6);

for i = 1:1:length(E)
    tree(i, :) = [G(1:3, i)', G(1:3, E(i))'];
end

createVideo(tree, 'filename', 2)