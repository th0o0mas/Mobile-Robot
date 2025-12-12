%% treeNode.m
classdef treeNode < handle
    properties (SetAccess = private)
        idx
        parent
        children
    end
     
    properties
        data  %position
        cost  %distance from root
    end
     
    
    methods
        function t = treeNode(varargin)
            if ~isempty(varargin)
                assert(mod(numel(varargin), 2) == 0, 'Invalid (Field,Value) pairs.');
                for i = 1:2:numel(varargin)
                    t.(varargin{i}) = varargin{i+1};
                end
            end
        end
        function rt = getRoot(t)
            if ~isempty(t.parent)
                rt = t.parent.getRoot;
            else
                rt = t;
            end
        end
         
        function ts = getLeaves(t)
            if ~isempty(t.children)
                ts = [];
            for i = 1:numel(t.children)
                ts = [ts, t.children(i).getLeaves];
            end
            else
                ts = t;
            end
        end
         
        function sibs = getSiblings(t)
            if isempty(t.parent)
                sibs = [];
            else
                pa = t.parent;
                sibs = pa.children(pa.children ~= t);
            end
        end
         
        function ts = getAncestors(t)
            if ~isempty(t.parent)
                ts = [t.parent, t.parent.getAncestors];
            else
                ts = [];
            end
        end
         
        function ts = getDescendants(t)
            ts = [];
            if ~isempty(t.children)
                ts = [ts, t.children];
                for i = 1:numel(t.children)
                    ts = [ts, t.children(i).getDescendants];
                end
            end
        end
         
        function n = nAncestors(t)
            if isempty(t.parent)
                n = 0;
            else
                n = 1 + t.parent.nAncestors;
            end
        end
         
        function n = nDescendants(t)
            if isempty(t.children)
                n = 0;
            else
                n = numel(t.children);
                for i = 1:n
                    n = n + t.children(i).nDescendants;
                end
            end
        end
         
        function n = nNodes(t)
            n = t.getRoot.nDescendants + 1;
        end
         
        function d = depth(t)
            if ~isempty(t.parent)
                d = 1 + t.parent.depth;
            else
                d = 0;
            end
        end
         
        function child=addChild(t, data)
            child = treeNode('data',data,'parent',t,'idx',t.nNodes + 1);
            child.cost=norm((child.data-t.data))+t.cost;
            t.children = [t.children, child];
        end
        
        function ts = DFS(t)
            ts = t;
            if ~isempty(t.children)
                for i = 1:numel(t.children)
                    ts = [ts, t.children(i).DFS];
                end
            end

        end
         
        function ts = BFS(t)
            queue = t.getRoot;
            ts = [];
            while ~isempty(queue)
                tnow = queue(1);
                queue = [queue(2:end), tnow.children];
                ts = [ts, tnow];
            end
        end

        function node = get(t,idx)
            ts = t.IFS();
            if strcmp(idx, 'all')
                node = ts;
            else
                node = ts(idx);
            end
        end
         
        function set(t,idx,data)
            t.get(idx).data = data;
        end
         
        function child=addNode(t,idx,data)
            child = treeNode('data', data, 'parent', t.get(idx), 'idx', t.nNodes + 1);
            child.cost=norm((child.data-t.data))+t.cost;
            t.get(idx).children = [t.get(idx).children, child];
        end
         
        function idx = find(t,data)
            idx = [];
            ts = t.DFS;
            for i = 1:numel(ts)
                if ts(i).data == data
                idx = [idx, ts(i).idx];
                end
            end
        end
         
        function ts = IFS(t)
            ts = t.DFS;
            n = numel(ts);
            s = [{ts.idx}', arrayfun(@(x) {x}, 1:n)'];
            s = sortrows(s);
            ts = ts([s{:,2}]);

        end
        function BFSt(t)
            queue = t.getRoot;
            idx = 1;
            while ~isempty(queue)
                queue(1).idx = idx;
                queue = [queue(2:end), queue(1).children];
                idx = idx + 1;
            end
        end
         
        function chop(t, idx)
            cidx = t.get(idx).parent.children == t.get(idx);
            t.get(idx).parent.children(cidx) = [];
            t.BFSt;
        end
         
        function graft(t, idx, tin)
            tin.parent = t.get(idx);
            t.get(idx).children = [t.get(idx).children, tin];
            t.BFSt;
        end
        function updatecost(t)
            %update the cost for all the descendants of the node
            ts=t.getDescendants();
            for i=1:numel(ts)
                ts(i).cost=ts(i).parent.cost+norm(ts(i).parent.data-ts(i).data);
            end

        end
    end
    
end