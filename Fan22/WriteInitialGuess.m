function WriteInitialGuess(x, y, theta, v, a, phy, w, tf)
global params_
nfe = params_.opti.nfe;
N = params_.obs.num_grids;
lw = params_.vehicle.lw;
lf = params_.vehicle.lf;
lr = params_.vehicle.lr;
lb = params_.vehicle.lb;
xo = params_.obs.x(1:N);
yo = params_.obs.y(1:N);
Aobs = params_.obs.A(1:N);
Bobs = params_.obs.B(1:N);
Cobs = params_.obs.C(1:N);
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
    dx = xo(:)-x(ii);
    dy = yo(:)-y(ii);
    G1 = [st*dx-ct*dy-0.5*lb,-st*dx+ct*dy-0.5*lb,ct*dx+st*dy-(lf+lw),-ct*dx-st*dy-lr];
    f1 = [zeros(4,1);-1];
    A1 = [-G1,ones(N,1)];
    b1 = zeros(N,1);
    Aeq1 = [ones(1,4),0];
    beq1 = 1;
    lb1 = [zeros(4,1);-Inf];
    ub1 = [ones(4,1);Inf];
    [z1,~,flag1] = linprog(f1,A1,b1,Aeq1,beq1,lb1,ub1,options);
    if flag1 > 0 && ~isempty(z1)
        lambda0 = z1(1:4);
    else
        lambda0 = 0.25*ones(4,1);
    end
    VLocalX = [lf+lw;lf+lw;-lr;-lr];
    VLocalY = [0.5*lb;-0.5*lb;-0.5*lb;0.5*lb];
    vx = x(ii)+VLocalX*ct-VLocalY*st;
    vy = y(ii)+VLocalX*st+VLocalY*ct;
    G2 = vx*Aobs(:)'+vy*Bobs(:)'+ones(4,1)*Cobs(:)';
    f2 = [zeros(N,1);-1];
    A2 = [-G2,ones(4,1)];
    b2 = zeros(4,1);
    Aeq2 = [ones(1,N),0];
    beq2 = 1;
    lb2 = [zeros(N,1);-Inf];
    ub2 = [ones(N,1);Inf];
    [z2,~,flag2] = linprog(f2,A2,b2,Aeq2,beq2,lb2,ub2,options);
    if flag2 > 0 && ~isempty(z2)
        omega0 = z2(1:N);
    else
        omega0 = ones(N,1)/N;
    end
    for q = 1:N
        for r = 1:4
            fprintf(fid,'let Lambda[%g,%g,%g] := %.12g;\r\n',ii,r,q,lambda0(r));
        end
    end
    for q = 1:4
        for j = 1:N
            fprintf(fid,'let Omega[%g,%g,%g] := %.12g;\r\n',ii,j,q,omega0(j));
        end
    end
end
fprintf(fid,'let tf := %.12g;\r\n',tf);
fclose(fid);
end