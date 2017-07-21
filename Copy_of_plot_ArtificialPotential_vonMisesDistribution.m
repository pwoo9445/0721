%--- 移動速度----------------------
vel.x= 3.0; % x成分
vel.y= 3.0; % y成分
%----------------------------------
%--- 動特性ポテンシャルの定数--------
kappa = 10.0;
alpha = 1.0;
beta  = 1.0;
sigma = 0.5;
%----------------------------------
%--- データ範囲の設定---------------
x_g = -3.0:0.05:3.0;
y_g = -3.0:0.05:3.0;
[X,Y] = meshgrid(x_g,y_g);
[theta,rho] = cart2pol(X,Y);
sizeX = length(x_g);
sizeY = length(y_g);
%---------------------------------

%---動特性ポテンシャル計算----------
[direction, speed] = cart2pol(vel.x,vel.y);
mu = ones(sizeY,sizeX)*direction;
C = 1/(2*pi*besseli(0,kappa));             % ベッセル関数を含む定数項
vonMises = C * exp(kappa*cos(theta - mu)); % フォンミーゼス分布の項
velTerm   = alpha*beta*speed*exp(-rho/(2*sigma))/(2*pi*sigma); % 速度と距離の項
U = vonMises.*velTerm;  % 動特性ポテンシャル
%----------------------------------

%---斥力ポテンシャル計算-----
A1 = -alpha*beta*speed/(4*(pi^2)*sigma*besseli(0,kappa));  % 1st term(constant)
A2 = exp(kappa*cos(theta-mu)-(rho/2/sigma));             % 2nd term
f_x = A1*A2.*(-cos(theta)/2/sigma + kappa*sin(theta).*sin(theta-mu)./rho);
f_y = A1*A2.*(-sin(theta)/2/sigma - kappa*cos(theta).*sin(theta-mu)./rho);
%----------------------------------

%--- 引力ポテンシャルの情報--------
x_goal = 0.0;
y_goal = 2.0;
alpha_goal = 50.0;
sigma_goal = 1.0;
%---------------------------------

%--- 引力ポテンシャル計算----------
[theta_goal,rho_goal] = cart2pol(X - x_goal,Y - y_goal);
U_goal = -alpha_goal/(2*pi*(sigma_goal^2))*exp(-rho_goal.^2/(2*(sigma_goal^2)));
% itvl= -5:0.1:5;
% contour(X,Y,U_goal,'LevelList',itvl);
% contourf(X,Y,U,'LevelList',itvl);
% caxis([-1,1])
% colorbar
%---------------------------------

%----- プロット(ポテンシャル場) ---
% figure
% itvl= 0:0.1:50;
% contour(X,Y,U,'LevelList',itvl);
% contourf(X,Y,U,'LevelList',itvl);
caxis([0,1])
itvl= -1:0.1:1;
contour(X,Y,U,'LevelList',itvl);
xlim([-0.5,2.5])
ylim([-0.5,2.5])
% xlim([-3.0,3.0]);
% ylim([-3.0,3.0]);
% caxis([-1,0]);

% surf(X,Y,U);
% shading flat

colorbar
hold on
%----------------------------------


%----- プロット(斥力ベクトル) ---
%x = -3:0.25:3;
%y = -3:0.25:3;
%[X,Y] = meshgrid(x,y);
%quiver(downsample(X,5),downsample(Y,5),downsample(f_x,5),downsample(f_y,5));
quiver(X,Y,f_x,f_y,5);
%  [f_x2,f_y2] = gradient(U);
% quiver(X,Y,-f_x2,-f_y2);
%----------------------------------
