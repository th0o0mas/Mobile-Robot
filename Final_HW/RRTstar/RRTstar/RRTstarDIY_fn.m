function RRTstarDIY_fn(app)
    % Optimized RRT* algorithm with intelligent sampling and efficient search
    % + Inflate occupancy map to account for robot size (quick method)
    close all
    load map2mat.mat

    % =======================
    % 1) Inflate occupancy map
    % =======================
    % Robot size as a circle
    robotW = 10;   % width  
    robotL = 15;   % length 

    inflationRadius = 0.5 * hypot(robotW, robotL);   % sqrt(W^2+L^2)/2
    inflationRadius = inflationRadius + 0.5;         % optional margin tunable if we want to go closer to the walls

    if isa(OccupiedMap, "binaryOccupancyMap") || isa(OccupiedMap, "occupancyMap")
        InflatedMap = copy(OccupiedMap);
        inflate(InflatedMap, inflationRadius);
    else
        error("OccupiedMap is of type %s; expected occupancyMap or binaryOccupancyMap.", class(OccupiedMap));
    end

    % =======================
    % 2) State space + validator
    % =======================
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = InflatedMap;               % use inflated map for collision checks
    sv.ValidationDistance = 0.01;
    ss.StateBounds = [xlimit; ylimit; [-pi pi]];

    mapxlimit = [xlimit(1); xlimit(2)];
    mapylimit = [ylimit(1); ylimit(2)];
    startpos  = StartPosition';
    endpos    = EndPosition';

    mmconnection  = 300;
    minconnection = 150;
    tolerance     = 5;
    iter          = 700;

    % ========== Optimization Parameters ==========
    goal_bias_prob = 0.15;
    solution_found = false;
    best_cost = inf;
    best_path_node = [];
    iterations_without_improvement = 0;
    patience = 30;
    w_turn = 50;  % turning penalty weight 

    % Initializing the tree
    root = treeNode('data', startpos, 'idx', 1, 'cost', 0);
    i = 0;
    wBar = waitbar(0,'Wait for Calculation');

    while i <= iter
        disp(i)
        progress = i/iter;
        waitbar(progress, wBar, 'Calculating');

        % Adaptive connection radius
        current_nodes = root.nNodes();
        base_radius = (-(i/iter)*(mmconnection-minconnection))+mmconnection;
        density_factor = max(0.5, min(1.5, 500/sqrt(max(current_nodes,1))));
        maxconnection = base_radius * density_factor;

        % Intelligent Sampling
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
                    rand_node = root.get(rand_node_idx);
                    rand_r = maxconnection * rand();
                    rand_theta = rand() * 2 * pi;
                    pos_rand = [rand_node.data(1) + rand_r * cos(rand_theta);
                               rand_node.data(2) + rand_r * sin(rand_theta)];
                else
                    pos_rand = [(mapxlimit(2)-mapxlimit(1))*rand() + mapxlimit(1);
                               (mapylimit(2)-mapylimit(1))*rand() + mapylimit(1)];
                end
            end

            % Bounds
            if pos_rand(1) < mapxlimit(1) || pos_rand(1) > mapxlimit(2) || ...
               pos_rand(2) < mapylimit(1) || pos_rand(2) > mapylimit(2)
                continue;
            end

            % Validity check (uses inflated map)
            if isStateValid(sv, [pos_rand' pi])
                sampling_success = true;
            end
        end

        if ~sampling_success
            i = i + 1;
            continue;
        end

        % Build node list
        z = root.BFS();

        % Vectorized distances
        all_positions = zeros(2, numel(z));
        for j = 1:numel(z)
            all_positions(:, j) = z(j).data;
        end
        dist_vec = sqrt(sum((all_positions - pos_rand).^2, 1));


        % Lazy collision checking (closest 20)
        %Near neighbor used to grow dynamically, has been changed to
        %improve efficency
        [sorted_dist, sort_idx] = sort(dist_vec);

        maxNN = min(numel(z), 20);

        % Preallocate neighbor list (as the same class as z)
        NearNeighbor = repmat(z(1), 1, maxNN);
        nnCount = 0;

        NearestNeighbor = [];
        discmp = inf;

        for j = 1:maxNN
            idx = sort_idx(j);
            tempdis = sorted_dist(j);

            if tempdis > maxconnection
                break;
            end

            isValid = isMotionValid(sv, [z(idx).data' pi], [pos_rand' pi]);
            if isValid
                nnCount = nnCount + 1;
                NearNeighbor(nnCount) = z(idx);

                if tempdis < discmp
                    NearestNeighbor = z(idx);
                    discmp = tempdis;
                end
            end
        end

        % Trim unused preallocated entries
        NearNeighbor = NearNeighbor(1:nnCount);

        % Assign parent for X_new
        temp_cmpcost = inf;
        father = [];
        for j = 1:numel(NearNeighbor)
            temp_cost = NearNeighbor(j).cost + edgeCostWithTurnPenalty(NearNeighbor(j), pos_rand, w_turn); %Changed to take into account sharp turns
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

        % Rewire
        for j = 1:numel(NearNeighbor)
            temp_cost = X_new.cost + edgeCostWithTurnPenalty(X_new, NearNeighbor(j).data, w_turn); %Changes for sharp turn
            if temp_cost < NearNeighbor(j).cost
                root.chop(NearNeighbor(j).idx);
                root.graft(X_new.idx, NearNeighbor(j));
                NearNeighbor(j).updatecost();
            end
        end

        root.updatecost();

        % OPT 6: solution tracking
        dist_to_goal = norm(endpos - pos_rand);
        if dist_to_goal <= tolerance
            endpos_node = X_new.addChild(endpos);

            if X_new.cost < best_cost
                best_cost = X_new.cost;
                best_path_node = endpos_node;
                iterations_without_improvement = 0;
                disp(['Improved solution found! Cost: ', num2str(best_cost)]);
            else
                iterations_without_improvement = iterations_without_improvement + 1;
            end

            solution_found = true;

            if iterations_without_improvement > patience && i > iter * 0.5
                disp('Converged to optimal solution');
                break;
            end
        end

        % Fallback at last iteration
        if i == iter
            if solution_found && ~isempty(best_path_node)
                endpos_node = best_path_node;
                disp('Using best solution found');
            else
                z = root.BFS();
                NearestNeighbor = [];
                NearNeighbor = [];
                cmpcost = inf;
                cmpcost2 = inf;

                for j = 1:numel(z)
                    tempcost = norm(z(j).data - endpos) + z(j).cost;
                    tempdis  = norm(z(j).data - endpos);
                    isValid  = isMotionValid(sv, [z(j).data' pi], [endpos' pi]);

                    if tempcost < cmpcost && tempdis < maxconnection && isValid
                        NearNeighbor = z(j);
                        cmpcost = tempcost;
                    end
                    if tempdis > maxconnection && tempcost < cmpcost2 && isValid
                        NearestNeighbor = z(j);
                        cmpcost2 = tempcost;
                    end
                end

                if isempty(NearNeighbor) && ~isempty(NearestNeighbor)
                    endpos_node = NearestNeighbor.addChild(endpos);
                elseif ~isempty(NearNeighbor)
                    endpos_node = NearNeighbor.addChild(endpos);
                else
                    i = i - 10;
                    disp('keep searching');
                    waitbar(0.99, wBar, 'Converging');
                    continue;
                end
                disp('Unsolved - using best effort');
            end
        end

        i = i + 1;
    end

    waitbar(1, wBar, 'Finish');
    pause(1);
    close(wBar);

    if solution_found && ~isempty(best_path_node)
        endpos_node = best_path_node;
    end

    % ========= Path extraction =========
    path = [];
    Z = getAncestors(endpos_node);
    for j = 1:numel(Z)
        path = [path; (Z(j).data)']; %#ok<AGROW>
    end

    if ~((path(1,1)==endpos_node.data(1,1)) && (path(1,2)==endpos_node.data(2,1)))
        path = [endpos_node.data'; path];
    end

    % ========= Path shortcutting =========
    minTurnRadius = 10; % <-- tune in your map units (bigger = smoother, harder to satisfy)
    path = shortcut_path(path, sv, minTurnRadius);

    % (Interpolation code unchanged)
    new_path = zeros(2*size(path,1)-1, 2);
    new_path(1,:)   = path(1,:);
    new_path(end,:) = path(end,:);
    for g = 2:(size(new_path,1)-1)
        new_path(g,:) = (mod(g,2)==0) * ((path(floor(g/2)+1,:) - path(floor(g/2),:))/2 + path(floor(g/2),:)) + ...
                        (mod(g,2)~=0) * path(floor((g+1)/2),:);
    end

    new_path_twice = zeros(2*size(new_path,1)-1, 2);
    new_path_twice(1,:)   = new_path(1,:);
    new_path_twice(end,:) = new_path(end,:);
    for g = 2:(size(new_path_twice,1)-1)
        new_path_twice(g,:) = (mod(g,2)==0) * ((new_path(floor(g/2)+1,:) - new_path(floor(g/2),:))/2 + new_path(floor(g/2),:)) + ...
                              (mod(g,2)~=0) * new_path(floor((g+1)/2),:);
    end

    new_path_triple = zeros(2*size(new_path_twice,1)-1, 2);
    new_path_triple(1,:)   = new_path_twice(1,:);
    new_path_triple(end,:) = new_path_twice(end,:);
    for g = 2:(size(new_path_triple,1)-1)
        new_path_triple(g,:) = (mod(g,2)==0) * ((new_path_twice(floor(g/2)+1,:) - new_path_twice(floor(g/2),:))/2 + new_path_twice(floor(g/2),:)) + ...
                               (mod(g,2)~=0) * new_path_twice(floor((g+1)/2),:);
    end

    path_f = flip(new_path_triple, 1);

    ConstrPoints = flip(path, 1);
    [~, Locb] = ismember(ConstrPoints, path_f, 'rows');
    ConIdx = Locb;
    curve = bspline_interpo_constraint(path_f, 0.01, ConstrPoints, ConIdx);

    % Visualization
    f1 = figure(1);
    show(OccupiedMap); % show original map for visualization (not inflated)
    hold on;
    visualizeTree(root);
    hold off;

    f2 = figure(2);
    show(OccupiedMap);
    hold on;
    visualizePath(endpos_node);
    hold on;

    cla(app.UIAxes_pathresult);
    copyobj(allchild(findobj(f2,'Type','axes')), app.UIAxes_pathresult);

   
     % So we keep the behavior you already end up with:
    targetposition = flip(path, 1);

    % =========================
    % FINAL COST 
    % =========================

    %Final cost of the exported path (targetposition is N x 2)
    if size(targetposition,1) >= 2
        seg = diff(targetposition, 1, 1);                  % (N-1)x2
        final_cost = sum(sqrt(sum(seg.^2, 2)));            % total Euclidean length
    else
        final_cost = 0;
    end

    % Display in command window
    fprintf('Final path cost (after shortcutting): %.4f\n', final_cost);

    % Display on the plotted path figure (Figure 2)
    figure(f2);
    title(sprintf('Final path (cost = %.4f)', final_cost));

    % Also annotate inside the axis (top-left)
    ax2 = gca;
    text(ax2, 0.02, 0.98, sprintf('Cost: %.4f', final_cost), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'w', 'Margin', 4);

    save("target_position", "targetposition");
    save("tree_result", "f1");
    save("path_result", "f2");
end

% ========== HELPER FUNCTIONS ==========

function pos = sample_informed_ellipsoid(start_pos, end_pos, best_cost, xlimit, ylimit)
    c_min = norm(end_pos - start_pos);
    x_center = (start_pos + end_pos) / 2;

    c_best = best_cost;
    if c_best < inf
        r1 = c_best / 2;
        r2_sq = c_best^2 - c_min^2;
        if r2_sq <= 0
            % degenerate; fallback
            pos = [(xlimit(2)-xlimit(1))*rand() + xlimit(1);
                   (ylimit(2)-ylimit(1))*rand() + ylimit(1)];
            return;
        end
        r2 = sqrt(r2_sq) / 2;

        theta = rand() * 2 * pi;
        r = sqrt(rand()) * r2;

        angle = atan2(end_pos(2) - start_pos(2), end_pos(1) - start_pos(1));
        R = [cos(angle), -sin(angle); sin(angle), cos(angle)];

        local_point = [ (rand()-0.5)*2*r1; r*sin(theta) ];
        pos = x_center + R * local_point;
    else
        pos = [(xlimit(2)-xlimit(1))*rand() + xlimit(1);
               (ylimit(2)-ylimit(1))*rand() + ylimit(1)];
    end
end

function shortened_path = shortcut_path(path, state_validator, minTurnRadius)
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
                % 1) collision check for the shortcut segment
                isValid = isMotionValid(state_validator, ...
                    [shortened_path(i,:) pi], [shortened_path(j,:) pi]);

                if ~isValid
                    continue;
                end

                % 2) curvature / turning-radius feasibility check at affected corners
                if ~shortcutCornersFeasible(shortened_path, i, j, minTurnRadius)
                    continue;
                end

                % Accept shortcut
                shortened_path = [shortened_path(1:i,:); shortened_path(j:end,:)];
                improved = true;
                break;
            end
            i = i + 1;
        end
    end
end

function ok = shortcutCornersFeasible(P, i, j, minTurnRadius)
    % This shortcut replaces ... i -> i+1 -> ... -> j-1 -> j ...
    % New corners that matter:
    %   corner at i:   (i-1, i, j) if i>1
    %   corner at j:   (i, j, j+1) if j < N

    N = size(P,1);
    ok = true;

    % Corner at i (previous point exists)
    if i > 1
        ok = ok && cornerIsFeasible(P(i-1,:), P(i,:), P(j,:), minTurnRadius);
        if ~ok, return; end
    end

    % Corner at j (next point exists)
    if j < N
        ok = ok && cornerIsFeasible(P(i,:), P(j,:), P(j+1,:), minTurnRadius);
        if ~ok, return; end
    end
end

function ok = cornerIsFeasible(p_prev, p_mid, p_next, minTurnRadius)
    % Enforce min turning radius using circumcircle radius of the 3 points.
    a = norm(p_mid - p_prev);
    b = norm(p_next - p_mid);
    c = norm(p_next - p_prev);

    % Degenerate cases: treat as feasible
    if a < 1e-9 || b < 1e-9 || c < 1e-9
        ok = true;
        return;
    end

    % If nearly collinear, radius is effectively infinite -> feasible
    s = 0.5*(a+b+c);
    A2 = s*(s-a)*(s-b)*(s-c);   % squared area via Heron's formula (up to scaling)
    if A2 <= 1e-12
        ok = true;
        return;
    end

    A = sqrt(A2);
    R = (a*b*c) / (4*A);        % circumradius = turning radius implied by the corner

    ok = (R >= minTurnRadius);
end
function theta = angle2D(v)
    theta = atan2(v(2), v(1));
end

function d = angdiff(a, b)
    % smallest signed angle difference a-b in [-pi, pi]
    d = atan2(sin(a-b), cos(a-b));
end

function c = edgeCostWithTurnPenalty(fatherNode, newPos, w_turn)
    % Base length
    c_len = norm(newPos - fatherNode.data);

    % Turning penalty requires a "grandparent"
    parent = fatherNode;
    grand = parent.parent;  % treeNode must have parent; if not, see note below

    if isempty(grand)
        c = c_len; % no turn penalty at root
        return;
    end

    v1 = parent.data - grand.data;
    v2 = newPos - parent.data;

    if norm(v1) < 1e-9 || norm(v2) < 1e-9
        c = c_len;
        return;
    end

    a1 = angle2D(v1);
    a2 = angle2D(v2);
    dtheta = abs(angdiff(a2, a1));  % [0..pi]

    % Penalize heading change (quadratic works well)
    c = c_len + w_turn * (dtheta^2);
end