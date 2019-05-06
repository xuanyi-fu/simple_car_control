function Cobs_cell = MinkowskiSum(A, Ocell ,q)
%%%%%%%%%INPUT%%%%%%%%%
%A is a 2xn array that contains the coordinates of the vertices in robot(0)
%Ocell is a cell array in which each elements contains a 2xm array that contains the coordinates of the vertices of the
%obstacle(in CCW order)    
%n: number of vertices
%q = [xt, yt, theta]' robot parameter
%%%%%%%%%OUTPUT%%%%%%%%
%Cobs_cell: C-obstacles as a cell array

theta = q(3);
R = ROT(theta);
%get the number of obstacles
cell_num = max(size(Ocell));
Cobs_cell = {};

%get the number of vertices in A
n = size(A,2);
%rotate the robot by theta
for ver = 1:n
    A(:,ver) = R*A(:,ver);
end
%Order the -A(0) with its smallest y in CCW order
A = CCWorder(-A,n);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for oi = 1:cell_num
    O = Ocell{oi};
    %get the number of vertices IN Ooi
    m= size(O,2);

    %Order the O with its smallest y in CCW order in case of a wrong order of input
    O = CCWorder(O,m);

    %initialization
    i=1;j=1;
    A(:,n+1) = A(:,1);A(:,n+2) = A(:,2);
    O(:,m+1) = O(:,1);O(:,m+2) = O(:,2);
    Cobs = [];
    %applying minkowski sum
    while size(Cobs,2) < n+m
        Cobs = [Cobs,A(:,i)+O(:,j)];
        if angle(A(:,i),A(:,i+1)) < angle(O(:,j),O(:,j+1))
            i = i+1;
        elseif angle(A(:,i),A(:,i+1)) > angle(O(:,j),O(:,j+1))
            j = j+1;
        else
            i = i+1;
            j = j+1;
        end
        %make sure i < n+1 or j < m+1
        if i > n+1 || j > m+1
            break;
        end
    end
        %remove the redundant points
        num = size(Cobs,2);
        iden = ones(2,num);
        for k = 1:num
            temp = Cobs(:,k);
            tempMatrix = temp.*iden;
            diff = tempMatrix - Cobs;
            index = find(sum(abs(diff),1) == 0);
            if size(index) > 1
                Cobs(:,index(2))=[];
            end
        end
        %updete the obs cell
        Cobs_cell{oi} = Cobs;
        
end%end of the first for loop

end