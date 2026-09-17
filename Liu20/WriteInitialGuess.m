function WriteInitialGuess(x, y, theta, v, a, phy, w, tf)
global params_
nfe = params_.opti.nfe;
N = params_.obs.num_grids;
lw = params_.vehicle.lw;
lf = params_.vehicle.lf;
lr = params_.vehicle.lr;
lb = params_.vehicle.lb;
Aobs = params_.obs.A(1:N);
Bobs = params_.obs.B(1:N);
Cobs = params_.obs.C(1:N);
farkas_eps = 0.01;
nbeta = N + 4;
options = optimoptions('linprog','Display','none');
fid = fopen('ig.INIVAL','w');
for ii = 1:nfe
    fprintf(fid,'let x[%g] := %.12g;\r\n',ii,x(ii));
    fprintf(fid,'let y[%g] := %.12g;\r\n',ii,y(ii));
    fprintf(fid,'let theta[%g] := %.12g;\r\n',ii,theta(ii));
    fprintf(fid,'let v[%g] := %.12g;\r\n',ii,v(ii));
    fprintf(fid,'let a[%g] := %.12g;\r\n',ii,a(ii));
    fprintf(fid,'let phi[%g] := %.12g;\r\n',ii,phy(ii));
    fprintf(fid,'let w[%g] := %.12g;\r\n',ii,w(ii));
    ct = cos(theta(ii));
    st = sin(theta(ii));
    Aveh = [st -ct;-st ct;ct st;-ct -st];
    bveh = [0.5*lb+x(ii)*st-y(ii)*ct;0.5*lb-x(ii)*st+y(ii)*ct;lf+lw+x(ii)*ct+y(ii)*st;lr-x(ii)*ct-y(ii)*st];
    Abar = [Aveh;Aobs(:) Bobs(:)];
    bbar = [bveh;-Cobs(:)];
    Aeq = [Abar';ones(1,nbeta)];
    beq = [0;0;1];
    [beta0,fval,exitflag] = linprog(bbar,[],[],Aeq,beq,zeros(nbeta,1),[],options);
    if exitflag <= 0 || isempty(beta0)
        beta0 = zeros(nbeta,1);
    elseif fval < -1e-12
        beta0 = beta0*max(1,1.1*farkas_eps/(-fval));
    end
    for jj = 1:nbeta
        fprintf(fid,'let beta[%g,%g] := %.12g;\r\n',ii,jj,beta0(jj));
    end
end
fprintf(fid,'let tf := %.12g;\r\n',tf);
fclose(fid);
end