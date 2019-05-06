%car tracking simulation
clear all;clc;
addpath('MinkowskiSum');
%initialize the simulation environment
A = [-.5 -.5 .5 .5;.5 -.5 -.5 .5];% defines the robot
q = [1 2 0];

O1 = [0 0 10 10;6 4 4 6];O2=[10 10 20 20;14 11 11 14];
Cobs = {O1,O2};
obs_cell = MinkowskiSum(A/2, Cobs ,q);
Infla_obs_cell = MinkowskiSum(A, Cobs ,q);

%set the initial position and goal position
mapbox=[20,20];NumNodes=400;stepsize=1;
qI.coord = [0;0]; qG.coord= [19.5;19.5];

%similize the environment
figure(1)
axis([0 mapbox(1) 0 mapbox(2)])
for i = 1:length(Infla_obs_cell)
    O = Infla_obs_cell{i};
    patch(O(1,:),O(2,:),'green');hold on
end

for i = 1:length(obs_cell)
    O = obs_cell{i};
    patch(O(1,:),O(2,:),'red');hold on
end



plot(qI.coord(1),qI.coord(2),'*b');plot(qG.coord(1),qG.coord(2),'*r');
% title('Environment')
% xlabel('x axis')
% ylabel('y axis')

%do the global planner
[path ,V ,E ,bool] = build_RRT(qI, qG, NumNodes, stepsize,mapbox, Infla_obs_cell)

%% 

% waitforbuttonpress;
patch(O1(1,:),O1(2,:),'yellow');hold on
patch(O2(1,:),O2(2,:),'yellow');hold on
for i = 1:size(path,2)-1
    x = [path(i).coord(1) path(i+1).coord(1)];
    y = [path(i).coord(2) path(i+1).coord(2)];
    plot(x,y,'-gs','LineWidth',2,...
    'MarkerSize',7,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    hold on
end
%do the local planner
num_iter = size(path,2);A_matrix ={};
Axs = [];
Ays = [];
Ts  = [0];
xend = [];
for iter = 1:num_iter-1
    if iter == 1
    qI = path(iter);
    else
    qI.coord = xend.';    
    end
    
    if iter < num_iter-1 
    qG = path(iter+1);
    qG_next = path(iter + 2);
    else
    qG = path(iter+1);
    qG_next = path(iter + 1);
    end
    [A, Ax, Ay, T, xend] = localplanner(qI.coord,qG.coord,qG_next.coord);
    A_matrix{iter} =A;
    
    Axs = [Axs,Ax.'];
    Ays = [Ays,Ay.'];
    Ts  = [Ts, Ts(end)+T];
end

Ts = Ts(2:end);
%% 
syms t real
h = Axs(:,3).'*[t^3;t^2;t;1];
h2 = Ays(:,3).'*[t^3;t^2;t;1];
dh = diff(h,t);
ddh = diff(dh,t);

h = matlabFunction(h);
h2 = matlabFunction(h2);
dh = matlabFunction(dh);
ddh = matlabFunction(ddh);
ts = 0:0.1:Ts(3)-Ts(2);

xs = [h(0:0.1:Ts(3)-Ts(2));h2(0:0.1:Ts(3)-Ts(2))];

plot(xs(1,:),xs(2,:),'-ro')



