function is_safe = IsCollisionFree()
global params_
is_safe = true;
% Obstacle vertices, excluding the repeated last vertex
Nobs = params_.obs.num_grids;
Obs = [params_.obs.x(1:Nobs)', params_.obs.y(1:Nobs)'];
for ii = 1:params_.opti.nfe
    V = CreateVehiclePolygon(params_.opti.x(ii), params_.opti.y(ii), params_.opti.theta(ii), 2);
    % Four vehicle vertices A, B, C, D
    Veh = [V.x(1),V.y(1);V.x(4),V.y(4);V.x(7),V.y(7);V.x(10),V.y(10)];
    if IsConvexPolygonCollision(Veh,Obs)
        is_safe = false;
        return;
    end
end
end

function flag = IsConvexPolygonCollision(P,Q)
flag = true;
R = {P,Q};
for kk = 1:2
    S = R{kk};
    for ii = 1:size(S,1)
        jj = mod(ii,size(S,1))+1;
        edge = S(jj,:)-S(ii,:);
        axis_ = [-edge(2),edge(1)];
        p1 = P*axis_';
        p2 = Q*axis_';
        if max(p1)<min(p2) || max(p2)<min(p1)
            flag = false;
            return;
        end
    end
end
end