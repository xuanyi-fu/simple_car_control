function [A, Ax, Ay, T, xend] = localplanner(qI,qG,qG_next)
% 1) compute a reference path using a polynomial in flat output space
% 2) track the path using feedback linearization
% boundary conditions in state space
% x0 = [qI; 0;.0001];
% xf = [qG; 0;.0001];
if size(qI,1) == 2
 x0 = [qI; 0;1];
else
 x0 = qI;
end
direction = (qG_next - qG)/norm(qG_next - qG);
theta = atan2(direction(2),direction(1))
if qG == qG_next
    theta = pi/4;
end
 xf = [qG; theta;1];
T = norm(qI(1:2)-qG) ;

%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%
% norm of initial and final velocity along desired path
% it determines how much the curves will bend
% and can be freely chosen
S.u1 = 1;S.u2 = 1;
S.l = 0.335;
% boundary conditions in flat output space 
y0 = car_h(x0);
yf = car_h(xf);
dy0 = x0(4)*[cos(x0(3)); sin(x0(3))]; % desired starting velocity
dyf = x0(4)*[cos(xf(3)); sin(xf(3))]; % desired end velocity

% compute path coefficients
A  = poly3_coeff(y0, dy0, yf, dyf, T);
Ax = A(1,:);
Ay = A(2,:);

% plot desired path
figure(1)
X = A*poly3([0:.01:T]);
plot(X(1,:), X(2,:), '-r','LineWidth',2)
hold on


%%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.A = A;
% gains
S.k = [1;1];

% perturb initial condition
xa = x0;% + [.25;.25;.5;.5];

% simulate system
[ts, xas] = ode45(@car_ode, [0 T], xa, [], S);

xend  = xas(end,:);

% visualize
% waitforbuttonpress;
figure(1)
plot(xas(:,1), xas(:,2), '--ob');
% legend('desired path', 'executed path')
title('graph of position')
xlabel('state x');
ylabel('state y');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function y = car_h(x)
% output function

y = x(1:2);
end

function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];
end
function f = dpoly3(t)
f = [ 3*t.^2; 2*t; ones(size(t)); zeros(size(t))];
end
function f = d2poly3(t)
f = [ 6*t; 2; zeros(size(t)); zeros(size(t))];
end
% function f = d3poly3(t)
% f = [60*t.^2; 24*t; 6; zeros(size(t)); zeros(size(t)); zeros(size(t))];
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ua = car_ctrl(t, xa, S)
% tracking control law

% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% xi = xa(end);
% get current output & current velocity
y = car_h(xa); 
dy = [cos(xa(3)); sin(xa(3))]*xa(4);

% error state
z1 = y - yd;
z2 = dy - dyd;

% virtual inputs
v = d2yd - S.k(1)*z1 -S.k(2)*z2;

% augmented inputs ua=(dxi, u2)
ua = [(v(2)*cos(xa(3)) - v(1)*sin(xa(3)))*S.l/xa(4)^2;
    v(1)*cos(xa(3)) + v(2)*sin(xa(3))];
end

function ua = car_ctrl_bs(t, x, S)
% tracking control law
k0 = .1;
k = .1;
% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
ddyd = S.A*d2poly3(t);

% xi = xa(end);
% get current output & current velocity
y = car_h(x); 
dy = [cos(x(3)); sin(x(3))]*x(4);

e  = y - yd;
de = dy - dyd;

z = -dyd + k0*e + dy;

R = [cos(x(3)) -sin(x(3));
     sin(x(3)) cos(x(3))];
 
rhs = R.'*[1,0;0,1/(x(4)*x(4))]*(ddyd - e - k0*de - k*z);
u = zeros(2,1);

u(2) = rhs(1);
u(1) = rhs(2);

ua = u;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dxa = car_ode(t, xa, S)
% car ODE
xa = xa;% + normrnd(0,0.02,4,1); %inject gaussin noise; 
ua = car_ctrl(t, xa, S);

xi = xa(end);
dv = ua(2);
if abs(ua(1)) > tan(pi/5)
    disp('steer limit')
    if ua(1) < 0
        ua(1) = -tan(pi/5);
    else
        ua(1) = tan(pi/5);
    end
end

if abs(ua(2)) > 1
    if ua(2) < 0
        ua(2) = -1;
    else
        ua(2) = 1;
    end
end

dtheta = ua(1)*xa(4)/S.l;


dxa = [cos(xa(3))*xi;
       sin(xa(3))*xi;
       dtheta;
       dv];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
