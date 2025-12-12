function visualizePath(endnode)
    % Visualize the path from endnode to root
    
    hold on;
    
    % Set default node size and color
    node_size = 10;
    node_color = 'b';
    
    % Traverse the path backwards from endnode to root
    node = endnode;
    path = [];
    
    % Check for empty endnode and handle accordingly
    if isempty(node)
        disp('Path is empty. No node to visualize.');
        return;
    end
    
    % Iterate through valid parent nodes
    while ~isempty(node) 
        path = [path; node.data'];
        node = node.parent;
    end
    
    fprintf('Path size: %d\n', size(path, 1));
    % Plot the path and nodes
    plot(path(:, 1), path(:, 2), 'r-o', 'LineWidth', 2);
    plot(path(:, 1), path(:, 2), 'b-', 'LineWidth', 1);
    
    % Set axis limits and labels for the plot
    axis([0 200 0 200]);
    xlabel('X');
    ylabel('Y');
    title('RRT* Path');
    
    hold off;
end