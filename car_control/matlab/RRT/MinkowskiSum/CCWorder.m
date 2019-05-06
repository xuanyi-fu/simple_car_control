function O_ccw = CCWorder(O,m)
    %Order the O with its smallest y in CCW order in case of a wrong order of input
    %m is the number of vertices
    tO = [];
    tempO = O;
    [tO(1,:), tO(2,:)] = poly2ccw(tempO(1,:), tempO(2,:));
    tO = [tO,tO];
    [smallyO,smallyindexO] = find(tO(2,:) == min(tO(2,:)));
    %if there are two or more smallest y, we need to find the smallest x in
    %this case of ties. And then update the smallyindex
    if max(size(smallyindexO)) ~= 1
        [smallxO,smallyindexxO] = min(tO(1,smallyindexO));
    end
    smallyindexO = smallyindexO(smallyindexxO);
    for colO = 1:m
        O_ccw(:,colO) = tO(:,smallyindexO+colO-1);
    end
end