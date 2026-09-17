function SolveNLP()
global params_
!ampl rr.run
[params_.opti.x, params_.opti.y, params_.opti.theta, params_.opti.v, params_.opti.a, params_.opti.phi, params_.opti.w, params_.opti.terminal_time] = LoadAmplSolution();
end