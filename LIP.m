function LIP(zc, Tsup, sx, sy, xi, yi, vxi, vyi, px, py, a, b, theta) % a, b: params of costfunc
% zc, Tsup, a, b
% sx, sy are walk parameters, vectors
% xi, yi: initial CoM coordinate
% px, py: initial foot placement
% vxi, vyi: initial CoM velocity
% theta: changing *degrees* (not rad!) per step
n = 0;
g = 9.8;

sx = [sx, 0];
sy = [sy, 0]; % the step after the last step should be zero
theta = [theta, 0]; % theta(end) can be any value

px0 = px;
py0 = py; % desired foot placement

Tc = sqrt(zc / g);
C = cosh(Tsup / Tc);
S = sinh(Tsup / Tc);
D = a * (C - 1)^2 + b * (S / Tc)^2;

st = sind(theta);
ct = cosd(theta);

function R = Rotate(i)
    if i == 0 R = eye(2);
    else R = [ct(i), -st(i); st(i), ct(i)]; % rotation matrix, each time rotates theta degrees
    end
end

% what to write in the ducument:
% names of variables and their meanings; margin values for this loop
hold on
cal_foot_place(0);

while n < size(sx, 2)  % sx was expanded by one element previously
    %%%%%%%%%%%%% plot the n-th step trajectory
    t = 0;
    dt = Tsup / 100;
    xt = xi; vxt = vxi; yt = yi; vyt = vyi;

    for i = 0 : 100
        xt = [xt, (xi - px) * cosh(t / Tc) + Tc * vxi * sinh(t / Tc) + px];
        vxt = [vxt, (xi - px) / Tc * sinh(t / Tc) + vxi * cosh(t / Tc)];
        yt = [yt, (yi - py) * cosh(t / Tc) + Tc * vyi * sinh(t / Tc) + py];
        vyt = [vyt, (yi - py) / Tc * sinh(t / Tc) + vyi * cosh(t / Tc)];
        t = t + dt;
    end
    plot(xt, yt);
    hold on
    %%%%%%%%%%%%% update xi, yi, vxi, vyi for the next step
    vxi = vxt(end); vyi = vyt(end);
    xi = xt(end); yi = yt(end);
    %%%%%%%%%%%%% update n, the order number of the step
    n = n + 1;

    if n < size(sx, 2)
        if n == size(sx, 2) - 1
            a = 1;
            b = 1;
            D = a * (C - 1)^2 + b * (S / Tc)^2;
        end
        cal_foot_place(n); % calculate the actual foot place for step n
    end
end
%vxi
%vyi
function cal_foot_place(n) % nested function
    %%%%%%%%%%%%% calculate the desired foot place during the n-th step
    if n ~= 0 % note: rotating
        pxy0 = [px0; py0];
        sxy= [sx(n); -(-1)^n * sy(n)];
        pxy0 = pxy0 + Rotate(n) * sxy;
        px0 = pxy0(1);
        py0 = pxy0(2);
    end
    %%%%%%%%%%%%% calculate the coordinate (xbar, ybar)
    xybar = [sx(n + 1) / 2; (-1)^n * sy(n + 1) / 2];
    xybar = Rotate(n + 1) * xybar;
    xbar = xybar(1);
    ybar = xybar(2);
    vxybar = [(C + 1)/(Tc * S) * xbar; (C - 1)/(Tc * S) * ybar];
    vxybar = Rotate(0) * vxybar;
    vxbar = vxybar(1);
    vybar = vxybar(2);
    %%%%%%%%%%%%% target state of CoM, of the n-th step
    xd = px0 + xbar;
    yd = py0 + ybar;
    vxd = vxbar;
    vyd = vybar;
    %%%%%%%%%%%%% update px, py to be the real foot place in step n
    %a,b are parameters for cost func
    px = - a * (C - 1) / D * (xd - C * xi - Tc * S * vxi)...
         - b * S / (Tc * D) * (vxd - S / Tc * xi - C * vxi);
    py = - a * (C - 1) / D * (yd - C * yi - Tc * S * vyi)...
         - b * S / (Tc * D) * (vyd - S / Tc * yi - C * vyi);
    plot(px0, py0, 'x');
    plot(px, py, 'o');
    %plot(xd, yd, '^');
end
end