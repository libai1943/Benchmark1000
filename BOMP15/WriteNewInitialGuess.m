function WriteNewInitialGuess()
global params_

nfe = params_.opti.nfe;
ngrids = params_.obs.num_grids;

fid = fopen('ig.INIVAL', 'w');

for ii = 1:nfe

    % Main trajectory variables
    fprintf(fid, 'let x[%g] := %.12g;\r\n', ii, params_.opti.x(ii));
    fprintf(fid, 'let y[%g] := %.12g;\r\n', ii, params_.opti.y(ii));
    fprintf(fid, 'let theta[%g] := %.12g;\r\n', ii, params_.opti.theta(ii));
    fprintf(fid, 'let v[%g] := %.12g;\r\n', ii, params_.opti.v(ii));
    fprintf(fid, 'let a[%g] := %.12g;\r\n', ii, params_.opti.a(ii));
    fprintf(fid, 'let phi[%g] := %.12g;\r\n', ii, params_.opti.phi(ii));
    fprintf(fid, 'let w[%g] := %.12g;\r\n', ii, params_.opti.w(ii));

    % J2 primal variable r
    for jj = 1:ngrids
        fprintf(fid, 'let r[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.r(ii,jj));
    end

    % J2 primal variables s and z
    for jj = 1:4
        fprintf(fid, 'let s[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.s(ii,jj));

        fprintf(fid, 'let z[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.z(ii,jj));
    end

    % MAKKT multipliers for r >= 0
    for jj = 1:ngrids
        fprintf(fid, 'let lam_r[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.lam_r(ii,jj));
    end

    % MAKKT multipliers for s >= 0 and z >= 0
    for jj = 1:4
        fprintf(fid, 'let lam_s[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.lam_s(ii,jj));

        fprintf(fid, 'let lam_z[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.lam_z(ii,jj));
    end

    % MAKKT multipliers for Qp = b
    for jj = 1:4
        fprintf(fid, 'let nu[%g,%g] := %.12g;\r\n', ...
            ii, jj, params_.opti.nu(ii,jj));
    end

end

fprintf(fid, 'let tf := %.12g;\r\n', params_.opti.terminal_time);

fclose(fid);

end