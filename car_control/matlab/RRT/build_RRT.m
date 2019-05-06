function [path ,V ,E ,bool] = build_RRT(qI, qG, NumNodes, stepsize,mapbox, Cobs)
%%%% INPUTS %%%%%%
% qI and qG: initial and goal position of the robot, with size 2x1
% NumNodes: the limit number of nodes.
% mapbox denotes the dimension of world frame, with size 2x1;
% Cobs denotes the convex obstacles, it is a cell

%%%%%%%%OUTPUT%%%%%%%%%%
% the path connecting qI and qG 
% The set of vertices V and the set of edges E
%bool defines whether algorithm finds the succussful path
% if bool equals 1, there is a complete succussful path.
% if bool equals -1, there is a failure.
addpath('rrt')
qI.parent = 0;
qI.cost = 0;qG.cost = 0;

obstacle_num = max(size(Cobs));
x_max = mapbox(1);
y_max = mapbox(2);
E = {};
V(1) = qI;

for i = 1:NumNodes
    qrand = [floor(rand(1)*x_max);floor(rand(1)*y_max)];
    [qnear,idx] = Nearest_Vertex(qrand, V);
    
    qnew.coord = New_Conf(qnear,qrand,stepsize);
    qnew.parent = idx;
    
    bool = [];
    for i = 1:obstacle_num
        Q = Cobs{i};
        b = isintersect_linepolygon(qnew.coord, qnear.coord, Q);
        bool = [bool,b];
    end
    if min(min(bool)) == 0
        %collision, discard qnew
        continue;
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         qnew.cost = dist(qnear.coord,qnew.coord) + qnear.cost; %use l2 distance as cost
        qnew.cost = angle_cost(qnear,qnew.coord,V) + qnear.cost;
        
        Vnear = check_radius(V,qnew.coord);
        
        %check Vnear nodes collision
        collision_idx = [];
        for near_num1 = 1:length(Vnear)
            iscollision = [];
            qtmp = Vnear(near_num1);
            for i = 1:obstacle_num
                Q = Cobs{i};
                b = isintersect_linepolygon(qnew.coord, qtmp.coord, Q);
                iscollision = [iscollision,b];
            end
            if min(min(iscollision)) == 0
                %collision, discard qnew
                collision_idx = [collision_idx,near_num1];
            end
        end
        Vnear(collision_idx) = [];
        
        qmin = qnear;Cmin = qnew.cost;
        for near_num2 = 1:length(Vnear)
            qtmp = Vnear(near_num2);
            if dist(qtmp.coord,qnew.coord)+qtmp.cost < Cmin %use l2 distance as cost
%             if angle_cost(qtmp,qnew.coord, V)+qtmp.cost < Cmin %use ANGLE as cost
                qmin = qtmp;
                Cmin = qtmp.cost + dist(qtmp.coord,qnew.coord);%use l2 distance as cost
%                 Cmin = qtmp.cost + angle_cost(qtmp, qnew.coord, V);%use ANGLE distance as cost
            end
            for kk = 1:length(V)
                qV = V(kk);
                if dist(qV.coord, qmin.coord) == 0
                    qnew.parent = kk;
                    qnear = qmin;
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        V = [V, qnew];
        edge = [qnear.coord,qnew.coord];%qnear is qnew parent
        E{size(E,2)+1} = edge;
        figure(1)
        x = [qnear.coord(1) qnew.coord(1)];
        y = [qnear.coord(2) qnew.coord(2)];
        line(x,y);hold on
    end
    if norm(qnew.coord-qG.coord) == 0
        break;
    end
end
% Search backwards from goal to start to find the optimal least cost path
[qstar,idx] = Nearest_Vertex(qG.coord, V);
qG.parent = idx;
bool = [];

for i = 1:obstacle_num
    Q = Cobs{i};
    b = isintersect_linepolygon(qG.coord, qstar.coord, Q);
    bool = [bool,b];
end
if min(min(bool)) == 0
    %collision, discard qnew
    bool = 0;
    %fail to find a successful path
    path = 0;
else
    bool =1;
    V = [V, qG];
    edge = [qstar.coord,qG.coord];
    E{size(E,2)+1} = edge;
    
%     child_point = qG;parent_point = qG;path=[];
%     path = [path,child_point];
%     while norm(parent_point-qI) ~= 0
%         for j = 1:size(E,2)
%             temp = E{j};
%             if norm(child_point-temp(:,2)) == 0
%                 parent_point = temp(:,1)
%                 child_point = temp(:,1);
%                 continue;
%             end
%         end    %end of for loop
%         path = [path,parent_point];
%     end
%     path = fliplr(path);
    % find path based on q goal to start backward search
    q_end = qG;
    path =[qG];
    while q_end.parent ~= 0
        start = q_end.parent;
        q_end = V(start);
        path = [path q_end];
    end
%     path = [path qI];
    path = fliplr(path);
end


end
