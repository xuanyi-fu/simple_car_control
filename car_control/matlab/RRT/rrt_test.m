% rrt algorithm test
clear all;clc;
O1 = [4 6 6 4;4 4 6 6];O2=[1 3 3;4 2 4];
Cobs = {O1,O2};
mapbox=[10,10];NumNodes=90;stepsize=1;
qI.coord = [0;0]; qG.coord= [9.5;9.5];
[path ,V ,E ,bool] = build_RRT(qI, qG, NumNodes, stepsize,mapbox, Cobs)

figure(1)
axis([0 mapbox(1) 0 mapbox(2)])
patch(O1(1,:),O1(2,:),'yellow');hold on
patch(O2(1,:),O2(2,:),'yellow');hold on
plot(qI.coord(1),qI.coord(2),'ob');plot(qG.coord(1),qG.coord(2),'or');

% for i = 1:size(V,2)
%     plot(V.coord(1,i),V.coord(2,i),'*g');hold on
% end

for i = 1:size(path,2)-1
    x = [path(i).coord(1) path(i+1).coord(1)];
    y = [path(i).coord(2) path(i+1).coord(2)];
    line(x,y);hold on
end

