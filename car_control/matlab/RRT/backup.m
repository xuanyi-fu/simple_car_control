%car tracking simulation
clear all;clc;
addpath('MinkowskiSum');
%initialize the simulation environment
A = [-.25 -.25 .25 .25;.25 -.25 -.25 .25];% defines the robot
q = [1 2 0];

O1 = [0 0 5 5;5 3 3 5];O2=[5 5 10 10;8 7 7 8];O3=[8 7 8 9 10 9;8 7 6 6 7 8];
Cobs = {O1,O2};
Cobs = {O1};
Infla_obs_cell = MinkowskiSum(A, Cobs ,q);

%set the initial position and goal position
mapbox=[10,10];NumNodes=400;stepsize=1;
qI.coord = [0;0]; qG.coord= [9.5;9.5];

%similize the environment
figure(1)
axis([0 mapbox(1) 0 mapbox(2)])
for i = 1:length(Infla_obs_cell)
    O = Infla_obs_cell{i};
    patch(O(1,:),O(2,:),'green');hold on
end

patch(O1(1,:),O1(2,:),'yellow');hold on
patch(O2(1,:),O2(2,:),'yellow');hold on
% patch(O3(1,:),O3(2,:),'yellow');hold on
plot(qI.coord(1),qI.coord(2),'*b');plot(qG.coord(1),qG.coord(2),'*r');

%do the global planner
[path ,V ,E ,bool] = build_RRT(qI, qG, NumNodes, stepsize,mapbox, Infla_obs_cell)
for i = 1:size(path,2)-1
    x = [path(i).coord(1) path(i+1).coord(1)];
    y = [path(i).coord(2) path(i+1).coord(2)];
    plot(x,y,'-gs','LineWidth',2,...
    'MarkerSize',7,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    hold on
end