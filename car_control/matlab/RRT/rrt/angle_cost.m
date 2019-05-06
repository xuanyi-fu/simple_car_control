function degree_abs =  angle_cost(n, q, V)
index = n.parent;
if index == 0
    degree_abs = angle(q,n.coord);
else
    n_former = V(index);
    degree1 = angle(n_former.coord,n.coord);
    degree2 = angle(q,n.coord);
    degree_abs = abs(degree1-degree2);
end

end


function degree = angle(a,b)
%%%%%%INPUT
%%%a, b are 2x1 vector
%%%%%%OUTPUT
%%%degree: the CCW angle w.r.t x axis
Xaxis = [1,0];Yaxis = [0,1];
edge = b' - a';
index = acos(dot(Yaxis,edge)/norm(edge))/pi*180;
phi = acos(dot(Xaxis,edge)/norm(edge));

if index <= 90.000
    degree = phi/pi*180;
else
    degree = 360 - phi/pi*180;
end
end