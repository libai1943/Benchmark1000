function WriteInitialGuess(x, y, theta, v, a, phy, w, tf)
global params_

nfe = params_.opti.nfe;
N = params_.obs.num_grids;

% ============================================================
% Ref. [17] geometry-based initialization
%
% The paper defines the obstacle centroid as the arithmetic
% average of its vertices, and initializes the separating
% hyperplane using the line between the trajectory sample
% and the obstacle centroid.
%
% gamma = 0.75 is used in the parking example of the paper.
% ============================================================

gamma = 0.75;

% Do not include the repeated closing vertex
xc_obs = mean(params_.obs.x(1:N));
yc_obs = mean(params_.obs.y(1:N));

fid = fopen('ig.INIVAL', 'w');

for ii = 1:nfe

    % ========================================================
    % Main trajectory variables
    % ========================================================

    fprintf(fid, 'let x[%g] := %.12g;\r\n', ii, x(ii));
    fprintf(fid, 'let y[%g] := %.12g;\r\n', ii, y(ii));
    fprintf(fid, 'let theta[%g] := %.12g;\r\n', ii, theta(ii));
    fprintf(fid, 'let v[%g] := %.12g;\r\n', ii, v(ii));
    fprintf(fid, 'let a[%g] := %.12g;\r\n', ii, a(ii));
    fprintf(fid, 'let phi[%g] := %.12g;\r\n', ii, phy(ii));
    fprintf(fid, 'let w[%g] := %.12g;\r\n', ii, w(ii));

    % ========================================================
    % Ref. [17]: separating hyperplane initialization
    %
    % Hyperplane:
    %
    % lambda_x * X + lambda_y * Y = mu
    %
    % The normal points from obstacle centroid toward the
    % trajectory sample, therefore the vehicle tends to lie
    % in H+ and the obstacle in H-.
    % ========================================================

    dx = x(ii) - xc_obs;
    dy = y(ii) - yc_obs;

    dist = hypot(dx, dy);

    if dist > 1e-10

        % Unit normal vector
        lambda_x0 = dx / dist;
        lambda_y0 = dy / dist;

    else

        % Numerical fallback; normally impossible for a
        % collision-free initial trajectory
        lambda_x0 = cos(theta(ii));
        lambda_y0 = sin(theta(ii));

    end

    % --------------------------------------------------------
    % A point through which the initial separating line passes:
    %
    % p_sep = gamma * p_vehicle
    %       + (1-gamma) * p_obstacle_centroid
    %
    % gamma = 0.75 places it closer to the vehicle.
    % --------------------------------------------------------

    x_sep = gamma * x(ii) + (1-gamma) * xc_obs;
    y_sep = gamma * y(ii) + (1-gamma) * yc_obs;

    % Hyperplane offset
    mu0 = lambda_x0 * x_sep + lambda_y0 * y_sep;

    % ========================================================
    % Write auxiliary-variable initial guesses
    % ========================================================

    fprintf(fid, 'let lambda_x[%g] := %.12g;\r\n', ...
        ii, lambda_x0);

    fprintf(fid, 'let lambda_y[%g] := %.12g;\r\n', ...
        ii, lambda_y0);

    fprintf(fid, 'let mu[%g] := %.12g;\r\n', ...
        ii, mu0);

end

fprintf(fid, 'let tf := %.12g;\r\n', tf);

fclose(fid);

end