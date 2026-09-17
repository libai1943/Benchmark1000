function WriteBasicParameterFile()
global params_
delete('BasicParameters');
fid = fopen('BasicParameters', 'w');
fprintf(fid, '1 %f\r\n', params_.vehicle.lw);
fprintf(fid, '2 %f\r\n', params_.vehicle.lf);
fprintf(fid, '3 %f\r\n', params_.vehicle.lr);
fprintf(fid, '4 %g\r\n', params_.vehicle.lb);
fprintf(fid, '5 %f\r\n', params_.vehicle.vmax);
fprintf(fid, '6 %f\r\n', params_.vehicle.vmin);
fprintf(fid, '7 %f\r\n', params_.vehicle.amax);
fprintf(fid, '8 %f\r\n', params_.vehicle.amin);
fprintf(fid, '9 %f\r\n', params_.vehicle.phimax);
fprintf(fid, '10 %f\r\n', params_.vehicle.wmax);
fprintf(fid, '11 %f\r\n', params_.opti.nfe);
fprintf(fid, '12 %f\r\n', params_.obs.num_grids);
fprintf(fid, '13 %f\r\n', params_.task.x0);
fprintf(fid, '14 %f\r\n', params_.task.y0);
fprintf(fid, '15 %f\r\n', params_.task.theta0);
fprintf(fid, '16 %f\r\n', params_.task.xf);
fprintf(fid, '17 %f\r\n', params_.task.yf);
fprintf(fid, '18 %f\r\n', params_.task.thetaf);
fprintf(fid, '19 %f\r\n', params_.obs.area);
fclose(fid);
end