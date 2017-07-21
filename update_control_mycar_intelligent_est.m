function [mycar, U_r, U_f] = update_control_mycar_intelligent_est(mycar, sim, othercars,pathTranslated,ratioSpeed, car_nr, front_num)
% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = 30000; % desired velocity
T = 1.0; % Safe time headway
a = 4000; % maximum acceleration
b = 6000; %desired deceleration
delta = 4; %acceleration exponent
s0 = 1000; % minimum distance
l = 2500; % vehicle length
%============================================================

U_r = 0;
U_f = 0;

% UPDATE MY CAR INFORMATION
persistent first_flag_est
if isempty(first_flag_est)
    first_flag_est = true;
end
%----Initial acceleration for lane changing----
if first_flag_est
    first_flag_est = false;
    mycar.vel(1) = mycar.vel(1)*ratioSpeed;
end
%----------------------------------------------

%---- Control angular velocity: vel(2) --------
pos = predict_pos(mycar.pos, mycar.vel, sim.T);
targetDegree = get_tatgetTheta(pos,pathTranslated);
mycar.vel(2) = mycar.vel(2) + (targetDegree - pos(3))/sim.T; 
%----------------------------------------------

if car_nr > 2
    A1 = othercars.car{front_num}.vel(1)/v0;
    
    if othercars.car{front_num}.pos(1) - mycar.pos(1) < 0 % if there is no other car front of this car
        A3 = track.xmax - mycar.pos(1) + othercars.car{front_num}.pos(1) - l;
    else
        A3 = othercars.car{front_num}.pos(1) - mycar.pos(1) - l;
    end
    A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;
    
    mycar.vel(1) = mycar.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
    
    if mycar.vel(1) < 0
        mycar.vel(1) = 0;
    end
end

mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);
mycar = update_rfs(mycar, othercars);

% %--- グリッドの設定 by Yanagihara---------------
% x_grid = 0:200:100000;
% y_grid = -3500:200:3500;
% [X,Y] = meshgrid(x_grid,y_grid);
% [theta,rho] = cart2pol(X,Y);
% sizeX = length(x_grid);
% sizeY = length(y_grid);
% %---------------------------------

%--- 動特性ポテンシャルの定数 by Yanagihara--------
kappa = 5.0;
alpha = 200000000000;
beta  = 1.0;
sigma = 500;
%---------------------------------

% %---othercar（後方）の動特性ポテンシャル計算（１点のみ）by Yanagihara-----------
% [theta_me2other,rho_me2other] = cart2pol(mycar.pos(1) - othercars.car{car_nr}.pos(1),mycar.pos(2) - othercars.car{car_nr}.pos(2));
% [direction_est, speed_est] = cart2pol(othercars.car{car_nr}.vel(1)*cos(othercars.car{car_nr}.pos(3)*pi/180),othercars.car{car_nr}.vel(1)*sin(othercars.car{car_nr}.pos(3)*pi/180));
% C = 1/(2*pi*besseli(0,kappa)); % ベッセル関数を含む定数項
% vonMises = C * exp(kappa*cos(theta_me2other - direction_est)); % フォンミーゼス分布の項
% velTerm = alpha*beta*speed_est*exp(-rho_me2other/(2*sigma))/(2*pi*sigma); % 速度と距離の項
% U_r = vonMises*velTerm;  % 動特性ポテンシャル
% %-------------------------------------------

% %---othercar（前方）の動特性ポテンシャル計算（１点のみ）by Yanagihara-----------
% [theta_me2other,rho_me2other] = cart2pol(mycar.pos(1) - othercars.car{front_num}.pos(1),mycar.pos(2) - othercars.car{front_num}.pos(2));
% [direction_est, speed_est] = cart2pol(othercars.car{front_num}.vel(1)*cos(othercars.car{front_num}.pos(3)*pi/180),othercars.car{front_num}.vel(1)*sin(othercars.car{front_num}.pos(3)*pi/180));
% C = 1/(2*pi*besseli(0,kappa)); % ベッセル関数を含む定数項
% vonMises = C * exp(kappa*cos(theta_me2other - direction_est)); % フォンミーゼス分布の項
% velTerm = alpha*beta*speed_est*exp(-rho_me2other/(2*sigma))/(2*pi*sigma); % 速度と距離の項
% U_f = vonMises*velTerm;  % 動特性ポテンシャル
% %-------------------------------------------

% %---othercar（後方）の動特性ポテンシャル計算（全グリッド）by Yanagihara-----------
% [theta_grid2other,rho_grid2other] = cart2pol(X - othercars.car{car_nr}.pos(1),Y - othercars.car{car_nr}.pos(2));
% [direction_est, speed_est] = cart2pol(othercars.car{car_nr}.vel(1)*cos(othercars.car{car_nr}.pos(3)*pi/180),othercars.car{car_nr}.vel(1)*sin(othercars.car{car_nr}.pos(3)*pi/180));
% mu_est = ones(sizeY,sizeX)*direction_est;
% C = 1/(2*pi*besseli(0,kappa)); % ベッセル関数を含む定数項
% vonMises = C * exp(kappa*cos(theta_grid2other - mu_est)); % フォンミーゼス分布の項
% velTerm = alpha*beta*speed_est*exp(-rho_grid2other/(2*sigma))/(2*pi*sigma); % 速度と距離の項
% U = vonMises.*velTerm;  % 動特性ポテンシャル
% %-------------------------------------------

% %---ポテンシャル場描画 by Yanagihara----------
% itvl= 0:2000:50000;
% flagPlot = true;
% if flagPlot
%     flagPlot = false;
%     [~,h.potential] = contour(X,Y,U,'LevelList',itvl);
% else
%     h.potential.XData = X;
%     h.potential.YData = Y;
%     h.potential.ZData = U;
% end
% 
% caxis([0,50000])
% colorbar
% 
% figure
% surf(X,Y,U);
% %--------------------------------------------

end

function targetDegree = get_tatgetTheta(pos,path)

nData = size(path,1);

dist=bsxfun(@hypot,path(:,1)-pos(1),path(:,2)-pos(2));
[~,idx]=min(dist);

if idx~=nData
   vx= path(idx+1,1)-path(idx,1); 
   vy= path(idx+1,2)-path(idx,2); 
else
   vx= path(idx,1)-path(idx-1,1); 
   vy= path(idx,2)-path(idx-1,2);   
end

targetDegree =atan(vy/vx)*180/pi;

end

function pos = predict_pos(pos, vel, T)

c = cos(pos(3)*pi/180);
s = sin(pos(3)*pi/180);
pos(1:2) = pos(1:2) + vel(1)*T*[c s];
pos(3) = pos(3) + vel(2)*T;

% DETERMINE DEGREE TO LIE BETWEEN -180~180
while pos(3) > 180
    pos(3) = pos(3) - 360;
end
while pos(3) < -180
    pos(3) = pos(3) + 360;
end

end


function mycar = update_rfs(mycar, othercars)
mycar.r = 0;
% EMULATE RFS
D2R    = pi/180;
nr_rfs = mycar.nr_rfs;
result = mycar.rfs_dist*ones(1, nr_rfs);
for i = 1:mycar.nr_rfs
    % FOR ALL RANGEFINDER BEAMS
    rfs_deg = mycar.pos(3) + mycar.rfs_degs(i);
    rfs_rad = rfs_deg * D2R;
    rfs_fr  = mycar.pos(1:2);
    rfs_to  = mycar.pos(1:2) ...
        + (mycar.r+mycar.rfs_dist)*[cos(rfs_rad) sin(rfs_rad)];
    min_dist2obs = mycar.rfs_dist;
    obs_boundary = zeros(2, 1E3);
    nr_obs_b = 0;
    for j = 1:othercars.n
        % FOR ALL OBSTACLES
        curr_obs_boundary = othercars.car{j}.bd;
        curr_obs_boundary = [curr_obs_boundary' [NaN ; NaN]];
        nr_curr_obs_b = size(curr_obs_boundary, 2);
        obs_boundary(:, nr_obs_b+1:nr_obs_b + nr_curr_obs_b) = curr_obs_boundary;
        nr_obs_b = nr_obs_b + nr_curr_obs_b;
    end
    obs_boundary = obs_boundary(:, 1:nr_obs_b);
    
    % Use InterX
    int_xy = InterX([rfs_fr(1) rfs_to(1) ; rfs_fr(2), rfs_to(2)], obs_boundary);
    if ~isempty(int_xy)
        xi = int_xy(1, :);
        yi = int_xy(2, :);
        for k = 1:length(xi)
            dist = norm([ mycar.pos(1)-xi(k), mycar.pos(2)-yi(k) ]);
            dist = dist - mycar.r;
            if dist < min_dist2obs
                min_dist2obs = dist;
            end
        end
    end
    result(i) = min_dist2obs;
end
mycar.rfs_dists = result;

end
