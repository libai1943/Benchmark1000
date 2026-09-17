param BasicParameters{i in {1..18}};
param lw := BasicParameters[1];
param lf := BasicParameters[2];
param lr := BasicParameters[3];
param lb := BasicParameters[4];
param vmax := BasicParameters[5];
param vmin := BasicParameters[6];
param amax := BasicParameters[7];
param amin := BasicParameters[8];
param phimax := BasicParameters[9];
param wmax := BasicParameters[10];
param nfe := BasicParameters[11];
param ngrids := BasicParameters[12];
param x0 := BasicParameters[13];
param y0 := BasicParameters[14];
param theta0 := BasicParameters[15];
param xf := BasicParameters[16];
param yf := BasicParameters[17];
param thetaf := BasicParameters[18];

param Obstacle{i in {1..ngrids}, k in {1..5}};
param dmin := 0.01;

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

var lambda_x{i in {1..nfe}};
var lambda_y{i in {1..nfe}};
var mu{i in {1..nfe}};

minimize obj: tf;

s.t. time_limit_common: tf <= 100;
s.t. DIFF_dxdt {i in {2..nfe}}: x[i] = x[i-1] + hi * v[i-1] * cos(theta[i-1]);
s.t. DIFF_dydt {i in {2..nfe}}: y[i] = y[i-1] + hi * v[i-1] * sin(theta[i-1]);
s.t. DIFF_dthetadt {i in {2..nfe}}: theta[i] = theta[i-1] + hi * tan(phi[i-1]) * v[i-1] / lw;
s.t. DIFF_dvdt {i in {2..nfe}}: v[i] = v[i-1] + hi * a[i-1];
s.t. DIFF_dphidt {i in {2..nfe}}: phi[i] = phi[i-1] + hi * w[i-1];
s.t. Bonds_phi {i in {1..nfe}}: -phimax <= phi[i] <= phimax;
s.t. Bonds_v {i in {1..nfe}}: vmin <= v[i] <= vmax;
s.t. Bonds_w {i in {1..nfe}}: -wmax <= w[i] <= wmax;
s.t. Bonds_a {i in {1..nfe}}: amin <= a[i] <= amax;
s.t. EQ_init_x: x[1] = x0;
s.t. EQ_init_y: y[1] = y0;
s.t. EQ_init_theta: theta[1] = theta0;
s.t. EQ_init_v: v[1] = 0;
s.t. EQ_init_phi: phi[1] = 0;
s.t. EQ_end_x: x[nfe] = xf;
s.t. EQ_end_y: y[nfe] = yf;
s.t. EQ_end_theta: theta[nfe] = thetaf;
s.t. EQ_end_v: v[nfe] = 0;
s.t. EQ_end_phi: phi[nfe] = 0;
s.t. EQ_end_w: w[nfe] = 0;
s.t. EQ_end_a: a[nfe] = 0;

s.t. SEP_VEHICLE {i in {1..nfe}, k in {1..4}}: lambda_x[i] * (x[i] + VLocalX[k]*cos(theta[i]) - VLocalY[k]*sin(theta[i])) + lambda_y[i] * (y[i] + VLocalX[k]*sin(theta[i]) + VLocalY[k]*cos(theta[i])) >= mu[i] + 0.5*dmin;
s.t. SEP_OBSTACLE {i in {1..nfe}, j in {1..ngrids}}: lambda_x[i] * Obstacle[j,1] + lambda_y[i] * Obstacle[j,2] <= mu[i] - 0.5*dmin;
s.t. SEP_NORMALIZATION {i in {1..nfe}}: lambda_x[i]^2 + lambda_y[i]^2 = 1;

data;
param: BasicParameters := include BasicParameters;
param: Obstacle := include Obstacle;