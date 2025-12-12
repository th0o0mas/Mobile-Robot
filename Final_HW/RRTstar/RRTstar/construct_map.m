% Create a figure and axes object
f = figure('Name','Map Occupation');
a = axes('Parent',f);

% Define map size
map_size = [20, 20];  % Adjust this to your desired size

% Create a blank white map image
map_image = repmat([255], map_size(1),map_size(2));

% Initialize variables
occupied_grids = zeros(map_size);
is_running = true;

% Define click callback function
click_callback = @(obj,eventdata) click_handler(obj,eventdata,a,map_image,occupied_grids,map_size,is_running);

% Set click event listener on the axes object
set(a,'ButtonDownFcn',click_callback);

% Function to handle click event
function click_handler(obj,eventdata,a,map_image,occupied_grids,map_size,is_running)
    while is_running
        % Get click coordinates
        [x_click, y_click] = ginput(1);

        % Check if click is within map bounds
        if x_click < 0 || x_click > map_size(2) || y_click < 0 || y_click > map_size(1)
            disp('Click outside map boundaries.');
            continue;
        end

        % Calculate grid index
        [grid_row, grid_col] = get_grid_index(x_click, y_click, map_size)

        % Check if grid is already occupied
        if occupied_grids(grid_row, grid_col) == 1
            disp('Grid already occupied!');
            continue;
        end

        % Check for exit key press
        if strcmpi(get(gcf, 'CurrentCharacter'), 'q')
            is_running = false;
            break;
        end

        % Change grid color to black and mark as occupied
        occupied_grids(grid_row, grid_col) = 1;
        map_image(grid_row, grid_col, :) = 0;

        % Update the image and display information
        image(map_image);
        drawnow;
        disp(['Grid clicked: (row, column) = (', num2str(grid_row), ', ', num2str(grid_col), ')']);
    end
end

% Function to get grid index from click coordinates
function [row, col] = get_grid_index(x_click, y_click, image_size)
    % Get image dimensions
    rows = image_size(1);
    cols = image_size(2);

    % Calculate grid size
    grid_width = image_size(2) / cols;
    grid_height = image_size(1) / rows;

    % Find grid indices
    col = floor(x_click / grid_width)+1 ;
    row = floor(y_click / grid_height) +1;
end
