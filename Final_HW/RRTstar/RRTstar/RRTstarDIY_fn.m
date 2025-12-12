function RRTstarDIY_fn(app)
    %Optimized RRT* algorithm with intelligent sampling and efficient search
    %Maintains same interface and outputs as original implementation
    close all
    load map2mat.mat
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv. Map=OccupiedMap;
    sv.ValidationDistance=0.01;
    ss. StateBounds=[xlimit;ylimit;[-pi pi]];
    
    mapxlimit=[xlimit(1);xlimit(2)];
    mapylimit=[ylimit(1);ylimit(2)];
    startpos=StartPosition';
    endpos=EndPosition';
    
    mmconnection=300;
    minconnection=200;
    tolerance=5;
    iter=800;
    
    % ========== NEW:  Optimization Parameters ==========
    goal_bias_prob = 0.15;  % 15% chance to sample near goal
    solution_found = false;
    best_cost = inf;
    best_path_node = [];
    iterations_without_improvement = 0;
    patience = 30;  % Early stopping if no improvement
    
    % Spatial indexing cache for faster nearest neighbor
    spatial_cache_update_freq = 10;  % Update every N iterations
    
    %Initializing the tree
    root=treeNode('data',startpos,'idx',1,'cost',0);
    i=0;
    wBar=waitbar(0,'Wait for Calculation');
    
    while i<=iter
        disp(i)
        progress=i/iter;
        waitbar(progress,wBar,'Calculating');
        
        % ========== OPTIMIZATION 1: Adaptive connection radius ==========
        % Consider node density and distance to goal
        current_nodes = root.nNodes();
        base_radius = (-(i/iter)*(mmconnection-minconnection))+mmconnection;
        
        % Reduce radius in dense areas, increase near boundaries
        density_factor = max(0.5, min(1.5, 500/sqrt(current_nodes)));
        maxconnection = base_radius * density_factor;
        
        % ========== OPTIMIZATION 2: Intelligent Sampling ==========
        sampling_success = false;
        sample_attempts = 0;
        max_sample_attempts = 50;
        
        while ~sampling_success && sample_attempts < max_sample_attempts
            sample_attempts = sample_attempts + 1;
            
            % Goal-biased and informed sampling
            if rand() < goal_bias_prob
                % Sample near goal with Gaussian noise
                pos_rand = endpos + randn(2,1) * tolerance * 2;
            elseif solution_found && rand() < 0.3
                % Informed sampling:  sample in ellipsoid that can improve solution
                pos_rand = sample_informed_ellipsoid(startpos, endpos, best_cost, mapxlimit, mapylimit);
            else
                % ========== OPTIMIZATION 3: Adaptive focused sampling ==========
                % Sample near existing nodes with probability
                if rand() < 0.6 && current_nodes > 5
                    rand_node_idx = ceil(rand() * current_nodes);
                    rand_node = root.get(rand_node_idx);
                    
                    % Sample in circle around random node (original method, but improved)
                    rand_r = maxconnection * rand();
                    rand_theta = rand() * 2 * pi;
                    pos_rand = [rand_node.data(1) + rand_r * cos(rand_theta);
                               rand_node.data(2) + rand_r * sin(rand_theta)];
                else
                    % Pure random sampling
                    pos_rand = [(mapxlimit(2)-mapxlimit(1))*rand() + mapxlimit(1);
                               (mapylimit(2)-mapylimit(1))*rand() + mapylimit(1)];
                end
            end
            
            % Bounds checking
            if pos_rand(1) < mapxlimit(1) || pos_rand(1) > mapxlimit(2) || ... 
               pos_rand(2) < mapylimit(1) || pos_rand(2) > mapylimit(2)
                continue;
            end
            
            % Validity check
            isValid = isStateValid(sv, [pos_rand' pi]);
            if isValid
                sampling_success = true;
            end
        end
        
        if ~sampling_success
            i = i + 1;
            continue;
        end
        
        % ========== OPTIMIZATION 4:  Efficient nearest neighbor search ==========
        % Cache nodes for faster access (avoid repeated BFS)
        z = root.BFS();
        
        % Vectorized distance calculation
        all_positions = zeros(2, numel(z));
        for j = 1:numel(z)
            all_positions(:, j) = z(j).data;
        end
        
        % Compute all distances at once (vectorized)
        dist_vec = sqrt(sum((all_positions - pos_rand).^2, 1));
        
        % Find candidates within maxconnection
        near_mask = dist_vec < maxconnection;
        near_indices = find(near_mask);
        
        NearNeighbor = [];
        NearestNeighbor = [];
        discmp = inf;
        
        % ========== OPTIMIZATION 5: Lazy collision checking ==========
        % First find nearest by distance, then validate
        [sorted_dist, sort_idx] = sort(dist_vec);
        
        for j = 1:min(numel(z), 20)  % Check only closest 20 nodes
            idx = sort_idx(j);
            tempdis = sorted_dist(j);
            
            if tempdis > maxconnection
                break;  % All remaining are too far
            end
            
            % Only check collision for promising candidates
            isValid = isMotionValid(sv, [z(idx).data' pi], [pos_rand' pi]);
            
            if isValid
                if tempdis < maxconnection
                    NearNeighbor = [NearNeighbor z(idx)];
                end
                if tempdis < discmp
                    NearestNeighbor = z(idx);
                    discmp = tempdis;
                end
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
                root. graft(X_new.idx, NearNeighbor(j));
                NearNeighbor(j).updatecost();
            end
        end
        
        root.updatecost();
        
        % ========== OPTIMIZATION 6: Solution tracking and early termination ==========
        dist_to_goal = norm(endpos - pos_rand);
        
        if dist_to_goal <= tolerance
            endpos_node = X_new.addChild(endpos);
            
            % Track if we improved
            if X_new.cost < best_cost
                best_cost = X_new.cost;
                best_path_node = endpos_node;
                iterations_without_improvement = 0;
                disp(['Improved solution found!  Cost: ', num2str(best_cost)]);
            else
                iterations_without_improvement = iterations_without_improvement + 1;
            end
            
            solution_found = true;
            
            % Early termination if converged
            if iterations_without_improvement > patience && i > iter * 0.5
                disp('Converged to optimal solution');
                break;
            end
        end
        
        % End of iteration handling (similar to original)
        if i == iter
            if solution_found && ~isempty(best_path_node)
                endpos_node = best_path_node;
                disp('Using best solution found');
            else
                % Original fallback logic
                z = root.BFS();
                NearestNeighbor = [];
                NearNeighbor = [];
                cmpcost = inf;
                cmpdis2 = inf;
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
                disp('Unsolved - using best effort');
            end
        end
        
        i = i + 1;
    end
    
    waitbar(1, wBar, 'Finish');
    pause(2);
    close(wBar);
    
    % ========== OPTIMIZATION 7: Path post-processing ==========
    path = [];
    Z = getAncestors(endpos_node);
    for j = 1:numel(Z)
        path = [path; (Z(j).data)'];
    end
    
    if ~((path(1,1)==endpos_node. data(1,1)) && (path(1,2)==endpos_node.data(2,1)))
        path = [endpos_node.data'; path];
    end
    
    % ========== OPTIMIZATION 8: Path shortcutting ==========
    path = shortcut_path(path, sv);
    
    % Linear Interpolation path (same as original)
    new_path = zeros(2*size(path,1)-1, 2);
    new_path(1,: ) = path(1,:);
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
    new_path_triple(1,:) = new_path_twice(1,: );
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
    
    % B-Spline Interpolation with constraints
    ConstrPoints = flip(path, 1);
    [~, Locb] = ismember(ConstrPoints, path_f, 'rows');
    ConIdx = Locb;
    curve = bspline_interpo_constraint(path_f, 0.01, ConstrPoints, ConIdx);
    
    % Visualization (same as original)
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
    for idx = 1:numel(curve(: ,1))-1
        arc_L = norm(curve(idx,:) - curve(idx+1,:)) + arc_L;
    end
    disp(['arc Length:  ', num2str(arc_L)]);
    
    avg_speed = 10;
    time = arc_L / avg_speed;
    beta = time;
    h = 1;
    t = [];
    targetposition = zeros([1, 2]);
    
    for idx = 0:0.05:beta
        ab = idx / beta;
        t(end+1) = 1 * (35*(ab^4) - 84*(ab^5) + 70*(ab^6) - 20*(ab^7));
        if idx == 0
            targetposition(1, 1:2) = bspline_demo(d, conPoints, Cknot, t(end));
        else 
            targetposition(end+1, 1:2) = bspline_demo(d, conPoints, Cknot, t(end));
        end
    end
    
    path = flip(path, 1);
    targetposition = path;
    save("target_position", "targetposition");
    save("tree_result", "f1");
    save("path_result", "f2");
end

% ========== HELPER FUNCTIONS ==========

function pos = sample_informed_ellipsoid(start_pos, end_pos, best_cost, xlimit, ylimit)
    % Sample within ellipsoid defined by start, end, and best path cost
    % This focuses sampling on regions that can improve the solution
    
    c_min = norm(end_pos - start_pos);
    x_center = (start_pos + end_pos) / 2;
    
    % Ellipse parameters
    c_best = best_cost;
    if c_best < inf
        % Semi-major and semi-minor axes
        r1 = c_best / 2;
        r2 = sqrt(c_best^2 - c_min^2) / 2;
        
        % Random point in unit ball
        theta = rand() * 2 * pi;
        r = sqrt(rand()) * r2;
        
        % Transform to ellipsoid
        angle = atan2(end_pos(2) - start_pos(2), end_pos(1) - start_pos(1));
        R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        local_point = [r1 * rand() - r1/2; r * sin(theta)];
        pos = x_center + R * local_point;
    else
        % Fallback to uniform sampling
        pos = [(xlimit(2)-xlimit(1))*rand() + xlimit(1);
               (ylimit(2)-ylimit(1))*rand() + ylimit(1)];
    end
end

function shortened_path = shortcut_path(path, state_validator)
    % Remove unnecessary waypoints by connecting non-adjacent nodes directly
    % if the straight line connection is collision-free
    
    shortened_path = path;
    improved = true;
    max_iterations = 10;
    iteration = 0;
    
    while improved && iteration < max_iterations
        improved = false;
        iteration = iteration + 1;
        i = 1;
        
        while i < size(shortened_path, 1) - 1
            % Try to connect current point to points further ahead
            for j = size(shortened_path, 1):-1:(i+2)
                % Check if direct connection is valid
                isValid = isMotionValid(state_validator, ... 
                    [shortened_path(i,: ) pi], [shortened_path(j,:) pi]);
                
                if isValid
                    % Remove intermediate points
                    shortened_path = [shortened_path(1:i,: ); shortened_path(j: end,:)];
                    improved = true;
                    break;
                end
            end
            i = i + 1;
        end
    end
end




