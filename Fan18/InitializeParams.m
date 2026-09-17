function InitializeParams()
global params_
params_.utility.colorpool = [237,28,36; 0,162,232; 255,127,39; 218,112,214; 255,192,203; 123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;
params_.utility.ego_vehicle_rgb = [0.00, 0.45, 0.74];
params_.utility.traj_dt_for_resample = 0.1;

params_.scenario.xmin = -15;
params_.scenario.xmax = 15;
params_.scenario.ymin = -10;
params_.scenario.ymax = 10;
params_.scenario.num_rand_grids = 8;
params_.scenario.scale_rand_grids = 1.5;

params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;

params_.vehicle.vmax = 5.0;
params_.vehicle.vmin = 0.0;
params_.vehicle.amax = 0.5;
params_.vehicle.amin = -0.5;
params_.vehicle.phimax = 0.42;
params_.vehicle.wmax = 0.15;

params_.opti.nfe = 200;
end