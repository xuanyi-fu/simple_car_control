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