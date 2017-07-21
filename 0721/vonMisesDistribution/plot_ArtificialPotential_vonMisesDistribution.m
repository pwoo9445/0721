%--- 移動速度----------------------
vel.x= 3.0; % x成分
vel.y= 3.0; % y成分
%----------------------------------
%--- 動特性ポテンシャルの定数--------
kappa =30.0;
alpha = 1.0;
beta  = 1.0;
sigma = 0.5;
%----------------------------------
%--- データ範囲の設定---------------
x = -3:0.05:3;
y = -3:0.05:3;
[X,Y] = meshgrid(x,y);
[theta,rho] = cart2pol(X,Y);
sizeX = length(x);
sizeY = length(y);
%----------------------------------

%---動特性ポテンシャル計算----------
[direction, speed] = cart2pol(vel.x,vel.y);
mu = ones(sizeY,sizeX)*direction;
C = 1/(2*pi*besseli(0,kappa));             % ベッセル関数を含む定数項
vonMises = C * exp(kappa*cos(theta - mu)); % フォンミーゼス分布の項
velTerm   = alpha*beta*speed*exp(-rho/(2*sigma))/(2*pi*sigma); % 速度と距離の項
U = vonMises.*velTerm;  % 動特性ポテンシャル
%----------------------------------

%----- プロット --------------------
%figure
itvl= 0:0.1:50;
contour(X,Y,U,'LevelList',itvl);
%contourf(X,Y,U,'LevelList',itvl);
caxis([0,1])
colorbar
%----------------------------------
