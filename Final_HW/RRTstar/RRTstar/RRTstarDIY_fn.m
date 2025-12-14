function RRTstarDIY_fn(app)
    % Optimized RRT* with spacing constraint (no nodes closer than tolerance)
    close all
    load map2mat.mat
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = OccupiedMap;
    sv.ValidationDistance = 0.1;
    ss.StateBounds = [xlimit; ylimit; [-pi pi]];

    mapxlimit = [xlimit(1); xlimit(2)];
    mapylimit = [ylimit(1); ylimit(2)];
    startpos = StartPosition';
    endpos   = EndPosition';

    mmconnection = 300;
    minconnection = 200;
    tolerance = 5;   % enforce min spacing between nodes
    iter = 500;

    % Optimization parameters
    goal_bias_prob = 0.15;
    solution_found = false;
    best_cost = inf;
    best_path_node = [];
    iterations_without_improvement = 0;
    patience = 30;

    % Initialize tree
    root = treeNode('data', startpos, 'idx', 1, 'cost', 0);
    i = 0;
    wBar = waitbar(0, 'Wait for Calculation');

    while i <= iter
        progress = i / iter;
        waitbar(progress, wBar, 'Calculating');

        % Get current nodes once (cached for this iteration)
        z = root.BFS();
        current_nodes = numel(z);
        all_positions = zeros(2, current_nodes);
        for j = 1:current_nodes
            all_positions(:, j) = z(j).data;
        end

        % Adaptive connection radius
        base_radius = (-(i/iter)*(mmconnection - minconnection)) + mmconnection;
        density_factor = max(0.5, min(1.5, 500 / sqrt(current_nodes)));
        maxconnection = base_radius * density_factor;

        % -------- Intelligent sampling with spacing check --------
        sampling_success = false;
        sample_attempts = 0;
        max_sample_attempts = 50;

        while ~sampling_success && sample_attempts < max_sample_attempts
            sample_attempts = sample_attempts + 1;

            if rand() < goal_bias_prob
                pos_rand = endpos + randn(2,1) * tolerance * 2;
            elseif solution_found && rand() < 0.3
                pos_rand = sample_informed_ellipsoid(startpos, endpos, best_cost, mapxlimit, mapylimit);
            else
                if rand() < 0.6 && current_nodes > 5
                    rand_node_idx = ceil(rand() * current_nodes);
                    rand_node = z(rand_node_idx);
                    rand_r = maxconnection * rand();
                    rand_theta = rand() * 2 * pi;
                    pos_rand = [rand_node.data(1) + rand_r * cos(rand_theta);
                                rand_node.data(2) + rand_r * sin(rand_theta)];
                else
                    pos_rand = [(mapxlimit(2)-mapxlimit(1))*rand() + mapxlimit(1);
                                (mapylimit(2)-mapylimit(1))*rand() + mapylimit(1)];
                end
            end

            % Bounds check
            if pos_rand(1) < mapxlimit(1) || pos_rand(1) > mapxlimit(2) || ...
               pos_rand(2) < mapylimit(1) || pos_rand(2) > mapylimit(2)
                continue;
            end

            % Validity check
            if ~isStateValid(sv, [pos_rand' pi])
                continue;
            end

            % Spacing check (reject if too close to any existing node)
            dist_vec = sqrt(sum((all_positions - pos_rand).^2, 1));
            if any(dist_vec <= tolerance)
                continue; % too close; resample
            end

            sampling_success = true;
        end

        if ~sampling_success
            i = i + 1;
            continue;
        end

        % -------- Nearest / near neighbors (vectorized) --------
        dist_vec = sqrt(sum((all_positions - pos_rand).^2, 1));
        near_mask = dist_vec < maxconnection & dist_vec > tolerance; % enforce spacing
        [sorted_dist, sort_idx] = sort(dist_vec);

        NearNeighbor = [];
        NearestNeighbor = [];
        discmp = inf;

        % Check only the closest candidates (lazy collision checking)
        max_checks = min(current_nodes, 20);
        for j = 1:max_checks
            idx = sort_idx(j);
            tempdis = sorted_dist(j);
            if tempdis > maxconnection
                break;
            end
            if ~isMotionValid(sv, [z(idx).data' pi], [pos_rand' pi])
                continue;
            end
            if tempdis < maxconnection
                NearNeighbor = [NearNeighbor z(idx)];
            end
            if tempdis < discmp
                NearestNeighbor = z(idx);
                discmp = tempdis;
            end
        end

        % Assign the parent for X_new
        temp_cmpcost = inf;
        father = [];
        for j = 1:numel(NearNeighbor)
            temp_cost = norm(NearNeighbor(j).data - pos_rand) + NearNeighbor(j).cost;
            if temp_cost < temp_cmpcost
                father = NearNeighbor(j);
                temp_cmpcost = temp_cost;
            end
        end

        if ~isempty(father)
            X_new = father.addChild(pos_rand);
        elseif ~isempty(NearestNeighbor)
            X_new = NearestNeighbor.addChild(pos_rand);
        else
            i = i + 1;
            continue;
        end

        root.BFSt();

        % Rewire the tree
        for j = 1:numel(NearNeighbor)
            temp_cost = norm(NearNeighbor(j).data - X_new.data) + X_new.cost;
            if temp_cost < NearNeighbor(j).cost
                root.chop(NearNeighbor(j).idx);
                root.graft(X_new.idx, NearNeighbor(j));
                NearNeighbor(j).updatecost();
            end
        end

        root.updatecost();

        % Track solutions and early stopping
        dist_to_goal = norm(endpos - pos_rand);

        if dist_to_goal <= tolerance
            endpos_node = X_new.addChild(endpos);
            if X_new.cost < best_cost
                best_cost = X_new.cost;
                best_path_node = endpos_node;
                iterations_without_improvement = 0;
            else
                iterations_without_improvement = iterations_without_improvement + 1;
            end
            solution_found = true;

            if iterations_without_improvement > patience && i > iter * 0.5
                break;
            end
        end

        % Fallback handling at final iteration (limit collision checks)
        if i == iter
            if solution_found && ~isempty(best_path_node)
                endpos_node = best_path_node;
            else
                % Use cached z/all_positions; take closest K to goal
                goal_dist = sqrt(sum((all_positions - endpos).^2, 1));
                [sorted_goal_dist, goal_idx] = sort(goal_dist);
                K = min(50, numel(goal_idx)); % limit expensive checks
                NearestNeighbor = [];
                NearNeighbor = [];
                cmpcost = inf;
                cmpcost2 = inf;

                for jj = 1:K
                    idx = goal_idx(jj);
                    node = z(idx);
                    tempcost = norm(node.data - endpos) + node.cost;
                    tempdis = sorted_goal_dist(jj);
                    isValid = isMotionValid(sv, [node.data' pi], [endpos' pi]);
                    if tempdis < maxconnection && tempcost < cmpcost && isValid
                        NearNeighbor = node;
                        cmpcost = tempcost;
                    end
                    if tempdis >= maxconnection && tempcost < cmpcost2 && isValid
                        NearestNeighbor = node;
                        cmpcost2 = tempcost;
                    end
                end

                if isempty(NearNeighbor) && (~isempty(NearestNeighbor))
                    endpos_node = NearestNeighbor.addChild(endpos);
                elseif ~isempty(NearNeighbor)
                    endpos_node = NearNeighbor.addChild(endpos);
                else
                    i = i - 10;
                    waitbar(0.99, wBar, 'Converging');
                    continue;
                end
            end
        end

        i = i + 1;
    end

    waitbar(1, wBar, 'Finish');
    pause(2);
    close(wBar);

    % Path extraction
    path = [];
    Z = getAncestors(endpos_node);
    for j = 1:numel(Z)
        path = [path; (Z(j).data)'];
    end
    if ~((path(1,1) == endpos_node.data(1,1)) && (path(1,2) == endpos_node.data(2,1)))
        path = [endpos_node.data'; path];
    end

    path = shortcut_path(path, sv);

    % Linear interpolations (unchanged)
    new_path = zeros(2*size(path,1)-1, 2);
    new_path(1,:) = path(1,:);
    new_path(end,:) = path(end,:);
    for g = 2:(size(new_path,1)-1)
        new_path(g,:) = (mod(g,2)==0) * ((path(floor(g/2)+1,:) - path(floor(g/2),:))/2 + path(floor(g/2),:)) + ...
                        (mod(g,2)~=0) * path(floor((g+1)/2),:);
    end

    new_path_twice = zeros(2*size(new_path,1)-1, 2);
    new_path_twice(1,:) = new_path(1,:);
    new_path_twice(end,:) = new_path(end,:);
    for g = 2:(size(new_path_twice,1)-1)
        new_path_twice(g,:) = (mod(g,2)==0) * ((new_path(floor(g/2)+1,:) - new_path(floor(g/2),:))/2 + new_path(floor(g/2),:)) + ...
                              (mod(g,2)~=0) * new_path(floor((g+1)/2),:);
    end

    new_path_triple = zeros(2*size(new_path_twice,1)-1, 2);
    new_path_triple(1,:) = new_path_twice(1,:);
    new_path_triple(end,:) = new_path_twice(end,:);
    for g = 2:(size(new_path_triple,1)-1)
        new_path_triple(g,:) = (mod(g,2)==0) * ((new_path_twice(floor(g/2)+1,:) - new_path_twice(floor(g/2),:))/2 + new_path_twice(floor(g/2),:)) + ...
                               (mod(g,2)~=0) * new_path_twice(floor((g+1)/2),:);
    end

    his_node = root.BFS();
    his_point = [];
    for j = 1:numel(his_node)
        his_point = [his_point; (his_node(j).data)'];
    end

    path_f = flip(new_path_triple, 1);

    ConstrPoints = flip(path, 1);
    [~, Locb] = ismember(ConstrPoints, path_f, 'rows');
    ConIdx = Locb;
    curve = bspline_interpo_constraint(path_f, 0.01, ConstrPoints, ConIdx);

    % Visualization (unchanged)
    f1 = figure(1);
    show(OccupiedMap);
    hold on;
    visualizeTree(root);
    hold off;

    f2 = figure(2);
    show(OccupiedMap);
    hold on;
    visualizePath(endpos_node);
    hold on;

    ax1 = findobj(f1, 'Type', 'axes');
    ax2 = findobj(f2, 'Type', 'axes');
    cla(app.UIAxes_pathresult);
    copyobj(allchild(ax2), app.UIAxes_pathresult);

    load Bspline_settings.mat
    arc_L = 0;
    for idx = 1:numel(curve(:,1))-1
        arc_L = norm(curve(idx,:) - curve(idx+1,:)) + arc_L;
    end
    disp(['arc Length:  ', num2str(arc_L)]);

    avg_speed = 10;
    time = arc_L / avg_speed;
    beta = time;
    t = [];
    targetposition = zeros([1, 2]);

    for idx = 0:0.05:beta
        ab = idx / beta;
        t(end+1) = 35*(ab^4) - 84*(ab^5) + 70*(ab^6) - 20*(ab^7);
        if idx == 0
            targetposition(1, 1:2) = bspline_demo(d, conPoints, Cknot, t(end));
        else
            targetposition(end+1, 1:2) = bspline_demo(d, conPoints, Cknot, t(end));
        end
    end

    path = flip(path, 1);
    targetposition = path; % keep same output behavior
    save("target_position", "targetposition");
    save("tree_result", "f1");
    save("path_result", "f2");
end

% ======== HELPER FUNCTIONS (unchanged APIs) ========
function pos = sample_informed_ellipsoid(start_pos, end_pos, best_cost, xlimit, ylimit)
    c_min = norm(end_pos - start_pos);
    x_center = (start_pos + end_pos) / 2;

    c_best = best_cost;
    if c_best < inf
        r1 = c_best / 2;
        r2 = sqrt(c_best^2 - c_min^2) / 2;

        theta = rand() * 2 * pi;
        r = sqrt(rand()) * r2;

        angle = atan2(end_pos(2) - start_pos(2), end_pos(1) - start_pos(1));
        R = [cos(angle), -sin(angle); sin(angle), cos(angle)];

        local_point = [r1 * rand() - r1/2; r * sin(theta)];
        pos = x_center + R * local_point;
    else
        pos = [(xlimit(2)-xlimit(1))*rand() + xlimit(1);
               (ylimit(2)-ylimit(1))*rand() + ylimit(1)];
    end
end

function shortened_path = shortcut_path(path, state_validator)
    shortened_path = path;
    improved = true;
    max_iterations = 10;
    iteration = 0;

    while improved && iteration < max_iterations
        improved = false;
        iteration = iteration + 1;
        i = 1;

        while i < size(shortened_path, 1) - 1
            for j = size(shortened_path, 1):-1:(i+2)
                if isMotionValid(state_validator, [shortened_path(i,:) pi], [shortened_path(j,:) pi])
                    shortened_path = [shortened_path(1:i,:); shortened_path(j:end,:)];
                    improved = true;
                    break;
                end
            end
            i = i + 1;
        end
    end
end