function qcoord = New_Conf(qnear,qrand,stepsize)
unit_direction = (qrand - qnear.coord)/norm(qrand - qnear.coord);
qcoord = qnear.coord + unit_direction*stepsize;
end