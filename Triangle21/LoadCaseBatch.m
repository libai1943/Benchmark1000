function LoadCaseBatch(case_id)
global params_
load case_data.mat
elem = test_case_data_set(case_id);
x = elem.x;
y = elem.y;

% Convex hull
k = convhull(x, y);
params_.obs.x = x(k);
params_.obs.y = y(k);

% Area of the convex polygon
params_.obs.area = polyarea(params_.obs.x, params_.obs.y);
params_.obs.num_grids = length(params_.obs.x) - 1;

params_.task.x0 = -10.0;
params_.task.y0 = 0.0;
params_.task.theta0 = 0.0;
params_.task.xf = 10.0;
params_.task.yf = 0.0;
params_.task.thetaf = 0.0;
end