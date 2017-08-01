function [othercars, dec] = update_othercars_mycar_intelligent_merge(othercars, sim, track, mycar, car_nr) % added by yanagihara

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = 10000; % desired velocity
T = 1.0; % Safe time headway
a = 4000; % maximum acceleration
b = 6000; %desired deceleration
delta = 4; %acceleration exponent
s0 = 1000; % minimum distance
l = 2500; % vehicle length
%============================================================


for i = 1:othercars.n
    
    front_num = i + 1;
    if mod(front_num, othercars.npl) == 1
        front_num = front_num - othercars.npl;
    end
    
    if mod(i, othercars.npl) ~= 0
        A1 = othercars.car{i}.vel(1)/v0;
        
        if i == car_nr % if the calculation target car is just behind of mycar
            if mycar.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
                A3 = track.xmax - othercars.car{i}.pos(1) + mycar.pos(1) - l;
            else
                A3 = mycar.pos(1) - othercars.car{i}.pos(1) - l;
            end
            A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
        else
            if othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
                A3 = track.xmax - othercars.car{i}.pos(1) + othercars.car{front_num}.pos(1) - l;
            else
                A3 = othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) - l;
            end
            A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;
        end
        
        othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        dec = -a*(1 - A1^delta - A2^2);
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
