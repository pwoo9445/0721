function othercars = update_othercars_intelligent_merge(othercars, sim, track, idm) % added by yanagihara

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

for i = 1:othercars.n
    
    % 各レーンの先頭車両以外はIDMに従って速度更新
    if mod(i, othercars.npl) ~= 0
        front_num = i + 1;
        if mod(front_num, othercars.npl) == 1
            front_num = front_num - othercars.npl;
        end
        
        if othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
            A3 = track.xmax - othercars.car{i}.pos(1) + othercars.car{front_num}.pos(1) - l;
        else
            A3 = othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) - l;
        end
        A1 = othercars.car{i}.vel(1)/v0;
        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;
        othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        
        if othercars.car{i}.vel(1) < 0
            othercars.car{i}.vel(1) = 0;
        end
    end
    
    othercars.car{i}.pos ...
        = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);

    
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end
