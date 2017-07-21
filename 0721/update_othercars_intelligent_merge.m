function othercars = update_othercars_intelligent_merge(othercars, sim, track) % added by yanagihara

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = 30000; % desired velocity
T = 1.0; % Safe time headway
a = 4000; % maximum acceleration
b = 6000; %desired deceleration
delta = 4; %acceleration exponent
s0 = 1000; % minimum distance
l = 2500; % vehicle length
%============================================================

for i = 1:othercars.n
    
    front_num = i + 1;
    if mod(front_num, track.nr_seg) == 1
        front_num = front_num - track.nr_seg; 
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
    
    %         %test0706
    %         if i == 2
    %             othercars.car{2}.vel(1) = 27500;
    %         end
    
    othercars.car{i}.pos ...
        = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);

    
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end
