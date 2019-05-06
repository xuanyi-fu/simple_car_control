function Vnear = check_radius(V,q)

r = 7;
ndist = [];idx = [];
for j = 1:1:length(V)
    n = V(j);
    tmp = dist(n.coord, q);
    if tmp <= r
        ndist = [ndist tmp];
        idx = [idx,j];
    end
end
% find the nodes within radius r
Vnear = V(idx);
end