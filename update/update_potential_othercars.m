function othercars = update_potential_othercars(mycar, othercars, sim, car_nr) % added by yanagihara

% persistent flagPlot

%--- グリッドの設定 by Yanagihara---------------
% x_grid = 0:200:100000;
% y_grid = -3500:200:3500;
% [X,Y] = meshgrid(x_grid,y_grid);
% [theta,rho] = cart2pol(X,Y);
% sizeX = length(x_grid);
% sizeY = length(y_grid);
%---------------------------------

%--- 移動物体の動特性ポテンシャルの定数 by Yanagihara--------
kappa = 5.0;
alpha = 200000000000;
beta  = 1.0;
sigma = 500;
%---------------------------------

if mycar.pos(1) - othercars.car{car_nr}.pos(1) < 7500

% %---mycarと各グリッドの相対角度・位置計算 by Yanagihara----------
%              [theta_grid2me,rho_grid2me] = cart2pol(X - mycar.pos(1),Y - mycar.pos(2));
% %-----------------------------------------------------------------
%  
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
A3 = kappa*cos(theta_me2obs-direction_me)-(rho_me2obs/2/sigma);
F_x = A1*A2*(-cos(theta_me2obs)/2/sigma + kappa*sin(theta_me2obs)*sin(theta_me2obs-direction_me)/rho_me2obs);
%-----------------------------------------------------------------------------------

%---後方車両の位置と速度を更新 by Yanagihara------------------------------------------
othercars.car{car_nr}.vel(1) = othercars.car{car_nr}.vel(1) + sim.T*F_x;
%-----------------------------------------------------------------------------------


% flagPlot = true;

    %---ポテンシャル場描画 by Yanagihara----------
%     itvl= 0:2000:100000;
%     if flagPlot
%        flagPlot = false;
%        [~,h.potential] = contour(X,Y,U,'LevelList',itvl);
%     else
%        h.potential.XData = X;
%        h.potential.YData = Y;
%        h.potential.ZData = U;
%     end
%    
%     caxis([0,100000])
%     colorbar
    %--------------------------------------------
else
    othercars.car{car_nr}.vel(1) = 10000;
    
end


for i = 1:othercars.n
    othercars.car{i}.pos ...
        = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T); 
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end


