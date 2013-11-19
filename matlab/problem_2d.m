addpath('./');
addpath('./kinematics');
addpath('./visualization');

% Set RRT parameters and select variant of RRT planner
rrt_variant         = 'bidirectional'; 
numberOfSamples     = 500;
stepSize            = 0.05;
connectProbability  = 0.2;
errorThreshold      = 0.1;

% Set initial and goal configuration and link lengths
ql      = [2 2 1];
q_init  = [0.14; 1.45; 1.0];
q_goal  = [-2.5; -0.5; -2.1];

% Load scene data
file = 'file2.scene';
obst = load(file);     % data in format [tx, ty, theta, dim_x, dim_y]

% Initialize trees, store as array of size 3 x n
G1 = [q_init];   % grow tree from initial configuration
E1 = [1];

G2 = [q_goal];   % grow tree from final configuration
E2 = [1];

while(1)    % iterate until goal is found, i.e. trees are connected
    % Sample random configuration one for each trees
    q_rand1 = [rand(); rand(); rand()] * 2 * pi- pi;
    q_rand2 = [rand(); rand(); rand()] * 2 * pi- pi;
    
    % With probability connectProbability extend the trees towards the same
    % point and check if they meet.
    if(rand() > connectProbability)
        q_rand2 = q_rand1;
    end
    
    % Find nearest points in already grown trees
    index1 =  getNearest(G1', q_rand1);
    index2 =  getNearest(G2', q_rand2);
    q_near1 = G1(:, index1);
    q_near2 = G2(:, index2);
    
    % Move from q_near1 towards q_rand1 for tree 1
    dq11 = q_rand1(1) - q_near1(1);
    dq21 = q_rand1(2) - q_near1(2);
    dq31 = q_rand1(3) - q_near1(3);
    
    % Move from q_near2 towards q_rand2 for tree 2
    dq12 = q_rand2(1) - q_near2(1);
    dq22 = q_rand2(2) - q_near2(2);
    dq32 = q_rand2(3) - q_near2(3);
    
    % Compute new node in both tree
    q_new1 = q_near1 + stepSize * [dq11; dq21; dq31];
    q_new2 = q_near2 + stepSize * [dq12; dq22; dq32];
    
    % Check for collisions
    collision1 = 0;
    collision2 = 0;
    
    for j = 1:1:size(obst,1)
        collision1 = collision3(q_new1, obst(j, 1:3), ql, obst(j, 4:5));
        
        if(collision1)
            break;
        end
    end
    
    for j = 1:1:size(obst,1)
        collision2 = collision3(q_new2, obst(j, 1:3), ql, obst(j, 4:5));
        
        if(collision2) 
            break;
        end
    end
    
    % Append configurations to trees
    if(~collision1)
        G1 = [G1, q_new1];
        E1(length(E1) + 1) = index1;       % store index of q_near as parent
    end
    
    if(~collision2)
        G2 = [G2, q_new2];
        E2(length(E2) + 1) = index2;       % store index of q_near as parent
    end
    
    if(norm(q_new1 - q_new2) < errorThreshold)
        disp(['Trees close enough found. Attempt connecting them ...']);
        break;
    end
    
    % Display updates
    if(~mod(length(G1) + length(G2), 50))
        disp(['Added node ', num2str(length(G1) + length(G2)), ' to trees']);
    end
end

%% Reconstruct tree

% Assemble both trees from the meeting point q_new1 and q_new2, i.e. the
% last nodes that were added to both trees
qSum1 = q_new1;
qSum2 = q_new2;

% Reconstruct path taken from q_init to q_new1
i = length(G1);

while(i > 1)
    i = E1(i);
    qSum1 = [qSum1 G1(1:3, i)];
end

% Reconstruct path taken from q_goal to q_new2
i = length(G2);

while(i > 1)
    i = E2(i);
    qSum2 = [qSum2 G2(1:3, i)];
end

% Append both arrays
qSum = [];
for i = length(qSum2):-1:1
    qSum(1:3, end + 1) = qSum2(1:3, i);
end

for i = 1:1:length(qSum1)
    qSum(1:3, end + 1) = qSum1(1:3, i);
end

drawScene(file, qSum', ql, 'test', fk(q_init, ql), fk(q_goal, ql));

%% Check if appending was good
for i = 1:1:length(qSum)-1
    error = norm(qSum(1:3, i) - qSum(1:3, i+1));
    if(error > 2*errorThreshold)
        disp(['Error at index: ', num2str(i), ', with error: ', ...
            num2str(error)]);
    end
end

%% Plot both trees

% Plot data in 3D space
plot3(G1(1,:), G1(2,:), G1(3,:), 'bx'); hold on;
plot3(G2(1,:), G2(2,:), G2(3,:), 'gx'); hold on;
grid on;

% Mark initial configuration
plot3(G1(1,1), G1(2,1), G1(3,1), 'ro', 'LineWidth',2, 'MarkerEdgeColor','b',...
                'MarkerFaceColor','r', 'MarkerSize',12)
            
% Mark goal configuration in case of goal directed RRTs
plot3(q_goal(1), q_goal(2), q_goal(3), 'go', 'LineWidth',2, 'MarkerEdgeColor','b',...
                'MarkerFaceColor','g', 'MarkerSize',12)
            
% Mark one of the connecting points
plot3(q_new1(1), q_new1(2), q_new1(3), 'go', 'LineWidth',2, 'MarkerEdgeColor','black',...
                'MarkerFaceColor','g', 'MarkerSize',12)

% Plot edges of trees
for i = 1:1:length(E1)
    X = [G1(1, i), G1(1, E1(i))];
    Y = [G1(2, i), G1(2, E1(i))];
    Z = [G1(3, i), G1(3, E1(i))];
    line(X, Y, Z, 'Color', 'b');
end

for i = 1:1:length(E2)
    X = [G2(1, i), G2(1, E2(i))];
    Y = [G2(2, i), G2(2, E2(i))];
    Z = [G2(3, i), G2(3, E2(i))];
    line(X, Y, Z, 'Color', 'g');
end

% Add labels to axes
xlabel('Joint angle 1 q_1','fontsize',14,'fontweight','b')
ylabel('Joint angle 2 q_2','fontsize',14,'fontweight','b')
zlabel('Joint angle 3 q_3','fontsize',14,'fontweight','b')

%% Prepare trees for video - call createVideo(tree1, 'filename', step, tree2)
tree1 = zeros(length(E1), 6);
tree2 = zeros(length(E2), 6);

for i = 1:1:length(E1)
    tree1(i, :) = [G1(1:3, i)', G1(1:3, E1(i))'];
end

for i = 1:1:length(E2)
    tree2(i, :) = [G2(1:3, i)', G2(1:3, E2(i))'];
end

createVideo(tree, 'filename', 5)