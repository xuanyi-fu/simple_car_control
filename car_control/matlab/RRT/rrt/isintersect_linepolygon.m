function b = isintersect_linepolygon(p0, p1, Q)
Q = [Q,Q(:,1),Q(:,2)];
if norm(p0-p1) == 0
    in = inpolygon(p0(1),p0(2),Q(1,:),Q(2,:));
    if numel(p0(in)) == 0
        b = 1;
    else
        b = 0;
    end
    
else
    tE = 0;tL = 1;
    ds = p1-p0;
    for i = 1:size(Q,2)-2
        ei = Q(:,i+1) - Q(:,i);
        ni = [ei(2),-ei(1)]/norm(ei);
        eii = Q(:,i+2) - Q(:,i+1);
        if dot(eii,ni) > 0
            ni = -ni;
        end %make sure ni is outward normal vector
        N = -dot((p0 - Q(:,i)),ni);
        D = dot(ds,ni);
        if D == 0
            if N <0
                b = 1;
                return;
            end
        end
        t=N/D;
        if D < 0
            tE = max(tE,t);
            if tE > tL 
                b=1;
                return;
            end
        else
            tL = min(tL,t);
            if tL < tE
                b = 1;
                return;
            end
        end
    end
    
    if tE < tL
        b = 0;
    else
        b = 1;
    end
end


end