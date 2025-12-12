% RRT* Algorithm with Custom `treeNode` Class

clc;
clear all;
close all;

% Map limits
mapxlimit = [0; 10];
mapylimit = [0; 10];

% Start and goal positions
startpos = [1; 1];
endpos = [5; 5];

% RRT parameters
max_connection = 2;
tolerance = 0.1;
max_iterations = 500;

% Create the root node
root = treeNode('data', startpos, 'cost', 0, 'idx', 1);

% Main loop
for i = 1:max_iterations
    % Generate random sample
    rand_pos = [(mapxlimit(2) - mapxlimit(1)) * rand(); (mapylimit(2) - mapylimit(1)) * rand()];

    % Find nearest neighbor
    nearest_node = root.get(root.find(nearestNeighbor(rand_pos)));

    % Extend a new node
    new_pos = steer(nearest_node.data, rand_pos, max_connection);

    % Check if the new node is outside the map limits
    if any(new_pos < [mapxlimit(1); mapylimit(1)]) || any(new_pos > [mapxlimit(2); mapylimit(2)])
        continue;
    end

    % Add new node to the tree
    new_node = nearest_node.addChild(new_pos);

    % Rewiring process
    rewire(root, new_node, max_connection, tolerance);

    % Check if goal is reached
    if norm(new_node.data - endpos) <= tolerance
        break;
    end
end

% Extract path
path = [endpos];
node = new_node;
while ~isempty(node.parent)
    path = [path; node.parent.data];
    node = node.parent;
end

% Visualize the tree and path
visualizeTree(root);
hold on;
plot(path(:, 1), path(:, 2), 'r-o', 'LineWidth', 2);
hold off;

% Display results
if i < max_iterations
    disp('Goal reached!');
else
    disp('Goal not reached within maximum iterations.');
end