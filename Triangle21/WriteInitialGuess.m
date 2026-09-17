function WriteInitialGuess(x, y, theta, v, a, phy, w, tf)
global params_

nfe = params_.opti.nfe;

lw = params_.vehicle.lw;
lf = params_.vehicle.lf;
lr = params_.vehicle.lr;
lb = params_.vehicle.lb;

fid = fopen('ig.INIVAL', 'w');

for ii = 1:nfe

    % Vehicle vertices
    AX = x(ii) + (lf+lw) * cos(theta(ii)) - 0.5*lb * sin(theta(ii));
    AY = y(ii) + (lf+lw) * sin(theta(ii)) + 0.5*lb * cos(theta(ii));
    BX = x(ii) + (lf+lw) * cos(theta(ii)) + 0.5*lb * sin(theta(ii));
    BY = y(ii) + (lf+lw) * sin(theta(ii)) - 0.5*lb * cos(theta(ii));
    CX = x(ii) - lr * cos(theta(ii)) + 0.5*lb * sin(theta(ii));
    CY = y(ii) - lr * sin(theta(ii)) - 0.5*lb * cos(theta(ii));
    DX = x(ii) - lr * cos(theta(ii)) - 0.5*lb * sin(theta(ii));
    DY = y(ii) - lr * sin(theta(ii)) + 0.5*lb * cos(theta(ii));

    % Main variables
    fprintf(fid, 'let x[%g] := %.10f;\r\n', ii, x(ii));
    fprintf(fid, 'let y[%g] := %.10f;\r\n', ii, y(ii));
    fprintf(fid, 'let theta[%g] := %.10f;\r\n', ii, theta(ii));
    fprintf(fid, 'let v[%g] := %.10f;\r\n', ii, v(ii));
    fprintf(fid, 'let a[%g] := %.10f;\r\n', ii, a(ii));
    fprintf(fid, 'let phi[%g] := %.10f;\r\n', ii, phy(ii));
    fprintf(fid, 'let w[%g] := %.10f;\r\n', ii, w(ii));

    % Vehicle vertex variables
    fprintf(fid, 'let AX[%g] := %.10f;\r\n', ii, AX);
    fprintf(fid, 'let AY[%g] := %.10f;\r\n', ii, AY);
    fprintf(fid, 'let BX[%g] := %.10f;\r\n', ii, BX);
    fprintf(fid, 'let BY[%g] := %.10f;\r\n', ii, BY);
    fprintf(fid, 'let CX[%g] := %.10f;\r\n', ii, CX);
    fprintf(fid, 'let CY[%g] := %.10f;\r\n', ii, CY);
    fprintf(fid, 'let DX[%g] := %.10f;\r\n', ii, DX);
    fprintf(fid, 'let DY[%g] := %.10f;\r\n', ii, DY);

end

fprintf(fid, 'let tf := %.10f;\r\n', tf);
fclose(fid);
end