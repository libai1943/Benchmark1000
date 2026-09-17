param BasicParameters{i in {1..19}};
param lw      := BasicParameters[1];
param lf      := BasicParameters[2];
param lr      := BasicParameters[3];
param lb      := BasicParameters[4];
param vmax    := BasicParameters[5];
param vmin    := BasicParameters[6];
param amax    := BasicParameters[7];
param amin    := BasicParameters[8];
param phimax  := BasicParameters[9];
param wmax    := BasicParameters[10];
param nfe     := BasicParameters[11];
param ngrids  := BasicParameters[12];
param x0      := BasicParameters[13];
param y0      := BasicParameters[14];
param theta0  := BasicParameters[15];
param xf      := BasicParameters[16];
param yf      := BasicParameters[17];
param thetaf  := BasicParameters[18];
param epsilon  := BasicParameters[19];

param Obstacle{i in {1..ngrids}, k in {1..2}};

param delta := 0.05;

param VLocalX{k in {1..4}} := if k <= 2 then lf+lw else -lr;
param VLocalY{k in {1..4}} := if k = 1 or k = 4 then 0.5*lb else -0.5*lb;

var tf >= 0.1;
var hi = tf / (nfe - 1);

var x{i in {1..nfe}};
var y{i in {1..nfe}};
var theta{i in {1..nfe}};
var v{i in {1..nfe}};
var a{i in {1..nfe}};
var phi{i in {1..nfe}};
var w{i in {1..nfe}};

# J2 primal variables p = [r, s, z]
var r{i in {1..nfe}, j in {1..ngrids}} >= 0;
var s{i in {1..nfe}, k in {1..4}} >= 0;
var z{i in {1..nfe}, k in {1..4}} >= 0;

# MAKKT multipliers for p >= 0
var lam_r{i in {1..nfe}, j in {1..ngrids}} >= 0;
var lam_s{i in {1..nfe}, k in {1..4}} >= 0;
var lam_z{i in {1..nfe}, k in {1..4}} >= 0;

# MAKKT multipliers for Qp = b
var nu{i in {1..nfe}, k in {1..4}};

minimize obj:
tf;

s.t. time_limit_common:
tf <= 100;

s.t. DIFF_dxdt {i in {2..nfe}}:
x[i] = x[i-1] + hi * v[i-1] * cos(theta[i-1]);

s.t. DIFF_dydt {i in {2..nfe}}:
y[i] = y[i-1] + hi * v[i-1] * sin(theta[i-1]);

s.t. DIFF_dthetadt {i in {2..nfe}}:
theta[i] = theta[i-1] + hi * tan(phi[i-1]) * v[i-1] / lw;

s.t. DIFF_dvdt {i in {2..nfe}}:
v[i] = v[i-1] + hi * a[i-1];

s.t. DIFF_dphidt {i in {2..nfe}}:
phi[i] = phi[i-1] + hi * w[i-1];

s.t. Bonds_phi {i in {1..nfe}}:
-phimax <= phi[i] <= phimax;

s.t. Bonds_v {i in {1..nfe}}:
vmin <= v[i] <= vmax;

s.t. Bonds_w {i in {1..nfe}}:
-wmax <= w[i] <= wmax;

s.t. Bonds_a {i in {1..nfe}}:
amin <= a[i] <= amax;

s.t. EQ_init_x:
x[1] = x0;

s.t. EQ_init_y:
y[1] = y0;

s.t. EQ_init_theta:
theta[1] = theta0;

s.t. EQ_init_v:
v[1] = 0;

s.t. EQ_init_phi:
phi[1] = 0;

s.t. EQ_end_x:
x[nfe] = xf;

s.t. EQ_end_y:
y[nfe] = yf;

s.t. EQ_end_theta:
theta[nfe] = thetaf;

s.t. EQ_end_v:
v[nfe] = 0;

s.t. EQ_end_phi:
phi[nfe] = 0;

s.t. EQ_end_w:
w[nfe] = 0;

s.t. EQ_end_a:
a[nfe] = 0;


# ============================================================
# Ref. [13]: J2 + MAKKT collision avoidance
# ============================================================

# c' p >= delta + epsilon
# c contains four ones corresponding to z1,z2,z3,z4
s.t. BOMP_J2_DISTANCE {i in {1..nfe}}:
sum {k in {1..4}} z[i,k] >= delta + epsilon;


# Q(x) p = b
# A*r - B*s + [z1,z2]' = 0
s.t. BOMP_QP_X {i in {1..nfe}}:
sum {j in {1..ngrids}} Obstacle[j,1]*r[i,j] - sum {k in {1..4}} (x[i] + VLocalX[k]*cos(theta[i]) - VLocalY[k]*sin(theta[i]))*s[i,k] + z[i,1] = 0;

s.t. BOMP_QP_Y {i in {1..nfe}}:
sum {j in {1..ngrids}} Obstacle[j,2]*r[i,j] - sum {k in {1..4}} (y[i] + VLocalX[k]*sin(theta[i]) + VLocalY[k]*cos(theta[i]))*s[i,k] + z[i,2] = 0;

s.t. BOMP_QP_R {i in {1..nfe}}:
sum {j in {1..ngrids}} r[i,j] + z[i,3] = 1;

s.t. BOMP_QP_S {i in {1..nfe}}:
sum {k in {1..4}} s[i,k] + z[i,4] = 1;


# ||c - lambda + Q' nu||^2 <= epsilon
s.t. BOMP_MAKKT_STATIONARITY {i in {1..nfe}}:
sum {j in {1..ngrids}}(-lam_r[i,j] + Obstacle[j,1]*nu[i,1] + Obstacle[j,2]*nu[i,2] + nu[i,3])^2 + sum {k in {1..4}}(-lam_s[i,k] - (x[i] + VLocalX[k]*cos(theta[i]) - VLocalY[k]*sin(theta[i]))*nu[i,1] - (y[i] + VLocalX[k]*sin(theta[i]) + VLocalY[k]*cos(theta[i]))*nu[i,2] + nu[i,4])^2 + (1-lam_z[i,1]+nu[i,1])^2 + (1-lam_z[i,2]+nu[i,2])^2 + (1-lam_z[i,3]+nu[i,3])^2 + (1-lam_z[i,4]+nu[i,4])^2 <= epsilon;


# lambda' p <= epsilon
s.t. BOMP_MAKKT_COMPLEMENTARITY {i in {1..nfe}}:
sum {j in {1..ngrids}} lam_r[i,j]*r[i,j] + sum {k in {1..4}} lam_s[i,k]*s[i,k] + sum {k in {1..4}} lam_z[i,k]*z[i,k] <= epsilon;


data;
param: BasicParameters := include BasicParameters;
param: Obstacle := include Obstacle;