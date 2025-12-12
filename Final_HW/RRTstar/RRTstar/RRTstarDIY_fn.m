function RRTstarDIY_fn(app)
    % Highly optimized RRT* with GPU acceleration and intelligent sampling
    % Adds:
    %  1) Flexible iterations (stop when improvement < threshold)
    %  2) Faster post-first-solution convergence via more informed sampling,
    %     reduced collision checks, and adaptive neighbor count.
    %  3) FIX: remove bspline_demo dependency on undefined variables (d, conPoints, Cknot)
    close all
    load map2mat.mat

    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = OccupiedMap;
    sv.ValidationDistance = 0.01;
    ss.StateBounds = [xlimit; ylimit; [-pi pi]];

    mapxlimit = [xlimit(1); xlimit(2)];
    mapylimit = [ylimit(1); ylimit(2)];
    startpos = StartPosition';
    endpos   = EndPosition';

    mmconnection  = 300;
    minconnection = 200;
    tolerance     = 6;
    iter          = 300;

    % ========== Optimization Parameters ==========
    solution_found = false;
    best_cost      = inf;
    best_path_node = [];
    best_iter      = inf;

    % Flexible early stop: if relative improvement < 1% for N successive improvements, stop
    improvement_threshold = 0.001;     % 1%
    min_iters_before_stop = 25;       % do not stop too early
    stall_window          = 12;       % successive small improvements to stop
    stall_count           = 0;

    % Post-solution speedup knobs
    goal_bias_pre  = 0.13;   % before first solution (explore)
    goal_bias_post = 0.35;   % after first solution (exploit)
    informed_bias_post = 0.75; % once solved, prefer informed sampling heavily
    local_bias_post    = 0.20; % still allow local expansion
    % remainder goes to uniform exploration

    % For rewiring & neighbor selection
    K_pre  = 60;   % neighbors checked before solution (more exploration)
    K_post = 25;   % neighbors checked after solution (faster iterations)
    max_sample_attempts_pre  = 60;
    max_sample_attempts_post = 25;

    % Rewire fewer nodes after solution to save time
    do_full_rewire_pre  = true;
    do_full_rewire_post = false;

    % ========== GPU Detection & Setup ==========
    useGPU = false;
    try
        gpu = gpuDevice; %#ok<NASGU>
        useGPU = true;
        disp(['✓ GPU detected: ', gpu.Name]);
    catch
        disp('✗ No GPU detected - using CPU with parfor acceleration');
    end

    % ========== Initialize tree ==========
    root = treeNode('data', startpos, 'idx', 1, 'cost', 0);
    i = 0;
    wBar = waitbar(0,'Wait for Calculation');

    while i <= iter
        progress = i / iter;
        if solution_found
            waitbar(progress, wBar, sprintf('Optimizing (Best: %.2f, stall:%d/%d)', best_cost, stall_count, stall_window));
        else
            waitbar(progress, wBar, 'Searching for solution.. .');
        end

        % ========== Adaptive connection radius ==========
        current_nodes  = root.nNodes();
        base_radius    = (-(i/iter)*(mmconnection-minconnection))+mmconnection;
        density_factor = max(0.5, min(1.5, 500/sqrt(max(current_nodes, 1))));
        maxconnection  = base_radius * density_factor;

        % ========== Choose sampling policy (faster after first solution) ==========
        if ~solution_found
            goal_bias_prob      = goal_bias_pre;
            max_sample_attempts = max_sample_attempts_pre;
        else
            goal_bias_prob      = goal_bias_post;
            max_sample_attempts = max_sample_attempts_post;
        end

        % ========== Intelligent Sampling ==========
        sampling_success = false;
        sample_attempts  = 0;

        while ~sampling_success && sample_attempts < max_sample_attempts
            sample_attempts = sample_attempts + 1;

            if ~solution_found
                % ===== BEFORE SOLUTION: exploration-heavy =====
                if rand() < goal_bias_prob
                    pos_rand = endpos + randn(2,1) * tolerance * 2;
                else
                    % Mix local expansion and global exploration
                    if rand() < 0.55 && current_nodes > 5
                        rand_node_idx = max(1, min(current_nodes, ceil(rand() * current_nodes)));
                        rand_node = root.get(rand_node_idx);
                        rand_r = maxconnection * rand();
                        rand_theta = rand() * 2*pi;
                        pos_rand = [rand_node.data(1) + rand_r*cos(rand_theta);
                                   rand_node.data(2) + rand_r*sin(rand_theta)];
                    else
                        pos_rand = [(mapxlimit(2)-mapxlimit(1))*rand() + mapxlimit(1);
                                   (mapylimit(2)-mapylimit(1))*rand() + mapylimit(1)];
                    end
                end
            else
                % ===== AFTER SOLUTION: exploit learned best path cost =====
                r = rand();
                if r < goal_bias_prob
                    % goal-directed refinement
                    pos_rand = endpos + randn(2,1) * max(1, tolerance*0.75);
                elseif r < goal_bias_prob + informed_bias_post
                    % informed sampling inside ellipsoid => fast improvements
                    pos_rand = sample_informed_ellipsoid(startpos, endpos, best_cost, mapxlimit, mapylimit);
                elseif r < goal_bias_prob + informed_bias_post + local_bias_post && current_nodes > 5
                    % local expansion near existing nodes to densify tree
                    rand_node_idx = max(1, min(current_nodes, ceil(rand() * current_nodes)));
                    rand_node = root.get(rand_node_idx);
                    rand_r = (0.6*maxconnection) * rand();
                    rand_theta = rand() * 2*pi;
                    pos_rand = [rand_node.data(1) + rand_r*cos(rand_theta);
                               rand_node.data(2) + rand_r*sin(rand_theta)];
                else
                    % small amount of uniform exploration
                    pos_rand = [(mapxlimit(2)-mapxlimit(1))*rand() + mapxlimit(1);
                               (mapylimit(2)-mapylimit(1))*rand() + mapylimit(1)];
                end
            end

            % Bounds
            if pos_rand(1) < mapxlimit(1) || pos_rand(1) > mapxlimit(2) || ...
               pos_rand(2) < mapylimit(1) || pos_rand(2) > mapylimit(2)
                continue;
            end

            % Validity check
            if isStateValid(sv, [pos_rand' pi])
                sampling_success = true;
            end
        end

        if ~sampling_success
            i = i + 1;
            continue;
        end

        % ========== Build BFS snapshot once ==========
        z = root.BFS();
        numNodes = numel(z);

        all_positions = zeros(2, numNodes);
        for j = 1:numNodes
            all_positions(:, j) = z(j).data;
        end

        % Adaptive K: fewer checks after solution -> faster iterations
        if ~solution_found
            K = min(numNodes, K_pre);
        else
            Kdyn = K_post;
            if isfinite(best_iter) && best_iter < iter
                Kdyn = max(10, round(K_post * max(0.5, 1 - (i - best_iter)/max(iter - best_iter, 1))));
            end
            K = min(numNodes, Kdyn);
        end

        % ========== GPU/CPU vectorized distance ==========
        if useGPU
            try
                all_positions_gpu = gpuArray(all_positions);
                pos_rand_gpu = gpuArray(pos_rand);
                diff_data = all_positions_gpu - pos_rand_gpu;
                dist_sq = sum(diff_data.^2, 1);
                [dist_sq_sorted_gpu, idx_gpu] = mink(dist_sq, K);
                dist_sorted = sqrt(gather(dist_sq_sorted_gpu));
                idx_sorted  = gather(idx_gpu);
            catch
                useGPU = false; % fallback to CPU
                diff_data = all_positions - pos_rand;
                dist_sq = sum(diff_data.^2, 1);
                [dist_sq_sorted, idx_sorted] = mink(dist_sq, K);
                dist_sorted = sqrt(dist_sq_sorted);
            end
        else
            diff_data = all_positions - pos_rand;
            dist_sq = sum(diff_data.^2, 1);
            [dist_sq_sorted, idx_sorted] = mink(dist_sq, K);
            dist_sorted = sqrt(dist_sq_sorted);
        end

        % ========== Collision checking ==========
        isValidVec = false(1, K);
        z_data = cell(1, K);
        for jj = 1:K
            if idx_sorted(jj) <= numNodes
                z_data{jj} = z(idx_sorted(jj)).data;
            end
        end

        parfor jj = 1:K
            if ~isempty(z_data{jj}) && dist_sorted(jj) <= maxconnection
                isValidVec(jj) = isMotionValid(sv, [z_data{jj}' pi], [pos_rand' pi]);
            end
        end

        % Collect valid neighbors
        NearNeighbor = [];
        NearestNeighbor = [];
        discmp = inf;

        for jj = 1:K
            if idx_sorted(jj) > numNodes
                break;
            end
            if dist_sorted(jj) > maxconnection
                break; % sorted, so the rest won't be feasible
            end
            if isValidVec(jj)
                idx = idx_sorted(jj);
                tempdis = dist_sorted(jj);
                NearNeighbor = [NearNeighbor z(idx)]; %#ok<AGROW>
                if tempdis < discmp
                    NearestNeighbor = z(idx);
                    discmp = tempdis;
                end
            end
        end

        % ========== Choose best parent (cost) ==========
        father = [];
        best_parent_cost = inf;

        % After solution found, consider fewer candidates (faster)
        if solution_found && numel(NearNeighbor) > 10
            tmp = zeros(1, numel(NearNeighbor));
            for j = 1:numel(NearNeighbor)
                tmp(j) = NearNeighbor(j).cost + norm(NearNeighbor(j).data - pos_rand);
            end
            [~, ord] = mink(tmp, 10);
            candidates = NearNeighbor(ord);
        else
            candidates = NearNeighbor;
        end

        for j = 1:numel(candidates)
            temp_cost = norm(candidates(j).data - pos_rand) + candidates(j).cost;
            if temp_cost < best_parent_cost
                father = candidates(j);
                best_parent_cost = temp_cost;
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

        % ========== Rewire (lighter after solution) ==========
        if (~solution_found && do_full_rewire_pre) || (solution_found && do_full_rewire_post)
            NN_for_rewire = NearNeighbor;
        else
            if numel(NearNeighbor) > 12
                dists = zeros(1, numel(NearNeighbor));
                for j = 1:numel(NearNeighbor)
                    dists(j) = norm(NearNeighbor(j).data - X_new.data);
                end
                [~, ord2] = mink(dists, 12);
                NN_for_rewire = NearNeighbor(ord2);
            else
                NN_for_rewire = NearNeighbor;
            end
        end

        for j = 1:numel(NN_for_rewire)
            temp_cost = norm(NN_for_rewire(j).data - X_new.data) + X_new.cost;
            if temp_cost < NN_for_rewire(j).cost
                root.chop(NN_for_rewire(j).idx);
                root.graft(X_new.idx, NN_for_rewire(j));
                NN_for_rewire(j).updatecost();
            end
        end

        root.updatecost();

        % ========== Solution tracking ==========
        dist_to_goal = norm(endpos - pos_rand);
        if dist_to_goal <= tolerance
            endpos_node = X_new.addChild(endpos);
            new_cost = X_new.cost;

            if ~solution_found
                solution_found = true;
                best_cost = new_cost;
                best_path_node = endpos_node;
                best_iter = i;
                stall_count = 0;
                disp(['✓ First solution found! Cost: ', num2str(best_cost)]);
            else
                if new_cost < best_cost
                    rel_impr = (best_cost - new_cost) / max(best_cost, eps);
                    best_cost = new_cost;
                    best_path_node = endpos_node;

                    if rel_impr < improvement_threshold && i >= min_iters_before_stop
                        stall_count = stall_count + 1;
                    else
                        stall_count = 0;
                    end

                    disp(['✓ Improved solution! Cost: ', num2str(best_cost), ...
                          ' | rel impr: ', num2str(rel_impr*100, '%.3f'), '%, stall: ', ...
                          num2str(stall_count), '/', num2str(stall_window)]);
                end
            end

            if solution_found && i >= min_iters_before_stop && stall_count >= stall_window
                disp('✓ Early stop: improvements below threshold consistently.');
                break;
            end
        end

        % ===== End of iteration handling (original fallback) =====
        if i == iter
            if solution_found && ~isempty(best_path_node)
                endpos_node = best_path_node;
                disp('✓ Using best solution found');
            else
                z = root.BFS();
                NearestNeighbor = [];
                NearNeighbor = [];
                cmpcost = inf;
                cmpcost2 = inf;

                for j = 1:numel(z)
                    tempcost = norm(z(j).data - endpos) + z(j).cost;
                    tempdis = norm(z(j).data - endpos);
                    isValid = isMotionValid(sv, [z(j).data' pi], [endpos' pi]);

                    if tempcost < cmpcost && tempdis < maxconnection && isValid
                        NearNeighbor = z(j);
                        cmpcost = tempcost;
                    end
                    if tempdis > maxconnection && tempcost < cmpcost2 && isValid
                        NearestNeighbor = z(j);
                        cmpcost2 = tempcost;
                    end
                end

                if isempty(NearNeighbor) && (~isempty(NearestNeighbor))
                    endpos_node = NearestNeighbor.addChild(endpos);
                elseif ~isempty(NearNeighbor)
                    endpos_node = NearNeighbor.addChild(endpos);
                else
                    i = i - 10;
                    disp('keep searching');
                    waitbar(0.99, wBar, 'Converging');
                    continue;
                end
                disp('✓ Unsolved - using best effort');
            end
        end

        i = i + 1;
    end

    waitbar(1, wBar, 'Finish');
    pause(0.5);
    close(wBar);

    if solution_found && ~isempty(best_path_node)
        endpos_node = best_path_node;
    end

    % ========== Path extraction ==========
    path = [];
    Z = getAncestors(endpos_node);
    for j = 1:numel(Z)
        path = [path; (Z(j).data)']; %#ok<AGROW>
    end

    if ~((path(1,1)==endpos_node.data(1,1)) && (path(1,2)==endpos_node.data(2,1)))
        path = [endpos_node.data'; path];
    end

    % ========== Path shortcutting ==========
    disp('Optimizing path.. .');
    path = shortcut_path(path, sv);

    % Linear Interpolation (original processing)
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

    % B-Spline Interpolation with constraints (kept)
    ConstrPoints = flip(path, 1);
    [~, Locb] = ismember(ConstrPoints, path_f, 'rows');
    ConIdx = Locb;
    curve = bspline_interpo_constraint(path_f, 0.01, ConstrPoints, ConIdx);

    % Visualization
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

    cla(app.UIAxes_pathresult);
    copyobj(allchild(findobj(f2, 'Type', 'axes')), app.UIAxes_pathresult);

    % ========== Length calculation ==========
    arc_L = 0;
    for idx = 1:(size(curve,1)-1)
        arc_L = arc_L + norm(curve(idx,:) - curve(idx+1,:));
    end
    disp(['arc Length: ', num2str(arc_L)]);

    % ========== FIX: remove dependency on undefined variables (d, conPoints, Cknot) ==========
    % Previously you generated targetposition via bspline_demo(...), but then overwrote it anyway.
    % Keeping the final behavior (targetposition is the path) and avoiding the crash:
    targetposition = flip(path, 1);

    save("target_position", "targetposition");
    save("tree_result", "f1");
    save("path_result", "f2");

    disp('✓ Optimization complete!');
end

% ========== HELPER FUNCTIONS ==========

function pos = sample_informed_ellipsoid(start_pos, end_pos, best_cost, xlimit, ylimit)
    c_min = norm(end_pos - start_pos);
    x_center = (start_pos + end_pos) / 2;

    c_best = best_cost;
    if c_best < inf
        r1 = c_best / 2;
        r2_sq = c_best^2 - c_min^2;

        if r2_sq > 0
            r2 = sqrt(r2_sq) / 2;
            theta = rand() * 2*pi;
            r = sqrt(rand()) * r2;

            angle = atan2(end_pos(2) - start_pos(2), end_pos(1) - start_pos(1));
            R = [cos(angle), -sin(angle); sin(angle), cos(angle)];

            local_point = [ (rand()-0.5) * 2*r1; r*sin(theta) ];
            pos = x_center + R * local_point;
        else
            t = rand();
            pos = start_pos + t*(end_pos - start_pos) + randn(2,1)*5;
        end
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
                isValid = isMotionValid(state_validator, ...
                    [shortened_path(i,:) pi], [shortened_path(j,:) pi]);

                if isValid
                    shortened_path = [shortened_path(1:i,:); shortened_path(j:end,:)];
                    improved = true;
                    break;
                end
            end
            i = i + 1;
        end
    end
end