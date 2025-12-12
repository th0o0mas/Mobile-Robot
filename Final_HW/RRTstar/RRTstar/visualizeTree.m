function visualizeTree(tree)
% Visualize the tree structure

hold on;

% Set default node size and color
node_size = 10;
node_color = 'b';

% Traverse the tree in depth-first order
function dfs_visualize(node)
    if isempty(node.children)
        % Leaf node
        plot(node.data(1), node.data(2), 'o', 'MarkerSize', node_size, 'MarkerFaceColor', node_color);
    else
        % Internal node
        plot(node.data(1), node.data(2), 'o', 'MarkerSize', node_size, 'MarkerFaceColor', node_color);
        for child = node.children
            % Draw line to child node
            line([node.data(1), child.data(1)], [node.data(2), child.data(2)], 'Color', 'k');
            dfs_visualize(child);
        end
    end
end

% Start visualization from root node
dfs_visualize(tree);

axis([0 200 0 200]);
xlabel('X');
ylabel('Y');
title('RRT* Tree');

hold off;
end