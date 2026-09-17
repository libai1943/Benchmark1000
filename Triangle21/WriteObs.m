function WriteObs()
global params_
delete('Obstacle');
fid = fopen('Obstacle', 'w');
for ii = 1 : params_.obs.num_grids
    fprintf(fid, '%d 1 %.10g\r\n', ii, params_.obs.x(ii));
    fprintf(fid, '%d 2 %.10g\r\n', ii, params_.obs.y(ii));
end
fclose(fid);
end