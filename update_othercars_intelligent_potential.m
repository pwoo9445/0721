function othercars = update_othercars_intelligent_potential(mycar, othercars, sim, track, car_nr) % added by yanagihara

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = 30000; % desired velocity
T = 1.0; % Safe time headway
a = 4000; % maximum acceleration
b = 6000; %desired deceleration
delta = 4; %acceleration exponent
s0 = 1000; % minimum distance
l = 2500; % vehicle length
%============================================================

%--- 移動物体の動特性ポテンシャルの定数 by Yanagihara--------
kappa = 5.0;
alpha = 200000000000;
beta  = 1.0;
sigma = 500;
%---------------------------------

% %---mycarの動特性ポテンシャル計算 by Yanagihara-----------
 [direction_me, speed_me] = cart2pol(mycar.vel(1)*cos(mycar.pos(3)*pi/180),mycar.vel(1)*sin(mycar.pos(3)*pi/180));             
%             mu_me = ones(sizeY,sizeX)*direction_me;
%             C = 1/(2*pi*besseli(0,kappa));             % ベッセル関数を含む定数項
%             vonMises = C * exp(kappa*cos(theta_grid2me - mu_me)); % フォンミーゼス分布の項
%             velTerm = alpha*beta*speed_me*exp(-rho_grid2me/(2*sigma))/(2*pi*sigma); % 速度と距離の項
%             U = vonMises.*velTerm;  % 動特性ポテンシャル
% %-------------------------------------------

[theta_me2obs,rho_me2obs] = cart2pol(othercars.car{car_nr}.pos(1) - mycar.pos(1),othercars.car{car_nr}.pos(2) - mycar.pos(2));

%---車線変更時、隣車線のすぐ後方車両がmycarから受ける斥力を計算 by Yanagihara-----------
A1 = -alpha*beta*speed_me/(4*(pi^2)*sigma*besseli(0,kappa));  % 1st term(constant)
A2 = exp(kappa*cos(theta_me2obs-direction_me)-(rho_me2obs/2/sigma));  % 2nd term
F_x = A1*A2*(-cos(theta_me2obs)/2/sigma + kappa*sin(theta_me2obs)*sin(theta_me2obs-direction_me)/rho_me2obs);
%-----------------------------------------------------------------------------------

%---隣車線のすぐ後方車両の位置と速度を更新 by Yanagihara------------------------------------------
othercars.car{car_nr}.vel(1) = othercars.car{car_nr}.vel(1) + sim.T*F_x;
%-----------------------------------------------------------------------------------


for i = 1:othercars.n
    if i < othercars.n
        front_num = i + 1;
        if front_num > othercars.n - 1
            front_num = 1;
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
        
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
    end
    
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end
