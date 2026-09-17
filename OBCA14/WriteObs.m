function WriteObs()
global params_

N = params_.obs.num_grids;
x = params_.obs.x;
y = params_.obs.y;

% A point guaranteed to be inside the convex polygon
xc = mean(x(1:N));
yc = mean(y(1:N));

A = zeros(1, N);
B = zeros(1, N);
C = zeros(1, N);

for ii = 1:N
    % Current edge: point ii -> point ii+1
    x1 = x(ii);
    y1 = y(ii);
    x2 = x(ii+1);
    y2 = y(ii+1);
    dx = x2 - x1;
    dy = y2 - y1;
    edge_length = hypot(dx, dy);

    % Unit normal vector
    A(ii) = dy / edge_length;
    B(ii) = -dx / edge_length;
    C(ii) = -(A(ii) * x1 + B(ii) * y1);

    % Make sure polygon interior satisfies:
    % A*x + B*y + C <= 0
    if A(ii) * xc + B(ii) * yc + C(ii) > 0
        A(ii) = -A(ii);
        B(ii) = -B(ii);
        C(ii) = -C(ii);
    end
end

% Store for later use, especially for OBCA dual-variable initialization
params_.obs.A = A;
params_.obs.B = B;
params_.obs.C = C;

% Write obstacle half-space parameters
fid = fopen('Obstacle', 'w');
for ii = 1:N
    fprintf(fid, '%d 1 %.10g\r\n', ii, A(ii));
    fprintf(fid, '%d 2 %.10g\r\n', ii, B(ii));
    fprintf(fid, '%d 3 %.10g\r\n', ii, C(ii));
end
fclose(fid);
end