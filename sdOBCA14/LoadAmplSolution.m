function [x, y, theta, v, a, phi, w, terminal_time] = LoadAmplSolution()
load([pwd, '\x.txt']);
load([pwd, '\y.txt']);
load([pwd, '\theta.txt']);
load([pwd, '\v.txt']);
load([pwd, '\a.txt']);
load([pwd, '\phi.txt']);
load([pwd, '\w.txt']);
load([pwd, '\terminal_time.txt']);
end