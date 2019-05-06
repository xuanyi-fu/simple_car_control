function [qnear,idx] = Nearest_Vertex(q, V)

% % Mdl = KDTreeSearcher(V);
% % Mdl = createns(V,'Distance','euclidean');
% IdxNN = knnsearch(V',q');
% qnear = V(:,IdxNN');
    ndist = [];
    for j = 1:1:length(V)
        n = V(j);
        tmp = dist(n.coord, q);% + angle_cost(n, q, V)/100
        ndist = [ndist tmp];
    end
    % find the nearest node
    [~, idx] = min(ndist);
    qnear = V(idx);
end