ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;

%========== Simulator Settings ================================ 
% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'TEST_STRAIGHT';
NR_LANE    = 2;
LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.02); % dt = 0.02 [sec]
othercars  = init_othercars();
%nr_cars    = randi([++1 4]);
nr_cars    = 3;  % 11
othercars  = test_addcars(othercars, track, nr_cars);
mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0));

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL DEMO';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;

% LANE CHANGE SETTING--------
FLAG_LANECHANGE = 0;
dx = 15000/3; % x-cood.interval of control points for bezier curve
ctlPt = [0 0; dx 0; 2*dx -LANE_WIDTH; 3*dx -LANE_WIDTH];
[laneChangePath, lengthP] = bezierCurve(ctlPt);
ratioSpeed = lengthP/15000*1.1;
%----------------------------
%============================================================

%--- 移動物体の動特性ポテンシャルの定数 by Yanagihara--------
kappa = 5.0;
alpha = 200000000000;
beta  = 1.0;
sigma = 500;
%---------------------------------

%--- グリッドの設定 by Yanagihara---------------
x_grid = 0:200:100000;
y_grid = -3500:200:3500;
[X,Y] = meshgrid(x_grid,y_grid);
[theta,rho] = cart2pol(X,Y);
sizeX = length(x_grid);
sizeY = length(y_grid);
%---------------------------------

flagPlot = true;

% RUN
% INITIALIZE SAVER
traj = init_traj(track, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed 
        case ''
        case {'leftarrow', 'semicolon'}
            %mycar.vel(2) = mycar.vel(2)+20;
            %mycar.vel(2) = mycar.vel(2)+10;
        case {'rightarrow', 'quote'}
            %mycar.vel(2) = mycar.vel(2)-20;
            %mycar.vel(2) = mycar.vel(2)-10;
        case {'uparrow', 'leftbracket'}
            %mycar.vel(1) = mycar.vel(1)+10000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)+5000;
        case {'downarrow', 'slash'}
            %mycar.vel(1) = mycar.vel(1)-10000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)-5000;
        case 'space'
            mycar.vel = [0 0];
        case {'1', '2', '3', '4', '5', '6'}
            nr_lane = str2num(key_pressed);
            mycar = set_mycar(mycar, get_posintrack(track, 1, 0, nr_lane, 0), [0 0]);
        case 's' % MAKE A CAR GO STRAIGHT
            mycar.pos(3) = 0; mycar.vel(2) = 0;
        case 'p'
            if isequal(sim.mode, 'RUN'), sim.mode = 'PAUSE'; 
            elseif isequal(sim.mode, 'PAUSE'), sim.mode = 'RUN'; end
        case 'q' 
            sim.mode = 'QUIT';
        %{
        case 'r'
            % RESET CAR CONFIGURATIONS
            othercars  = reset_othercars(othercars);
            nr_cars = randi([1 4]);
            othercars  = add_randomcars(othercars, track, nr_cars);
            mycar = set_mycar(mycar, get_posintrack(track, 1, 0, 1, 0), [0 0]);
            % RESET TRAJ AS CAR CONFIGURATIONS ARE CHANGED
            traj = init_traj(track, mycar, othercars);
        case 'w'
            % SAVE TO FILE
            savename = sprintf('data/raw_trajs/traj_%s.mat' ...
                , datestr(datenum(clock),'yyyymmdd_HHMM'));
            save(savename, 'traj');
            fprintf(2, 'TRAJ SAVE TO [%s]. \n', savename);
            sim.mode = 'QUIT';
        %}
        case 'r'
            % restart simulation (add by yanagihara)
            % INITIALIZE ENVIRONMENT
            TRACK_TYPE = 'TEST_STRAIGHT';
            NR_LANE    = 2;
            LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
            track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
            sim        = init_sim(0.02); % dt = 0.02 [sec]
            othercars  = init_othercars();
            othercars  = test_addcars(othercars, track, nr_cars);
            mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0));
            FLAG_LANECHANGE = 0;
        case 'l'
            % LANE CHANGING (add by kumano)
            FLAG_LANECHANGE = 1;
            FLAG_LANECHANGE_FINISHED = 0;
            %---get the number of the nearest car (front and rear) (by yanagihara)
            car_nr = othercars.n - 1;
            for i = 1:othercars.n - 1
                if mycar.pos(1) - othercars.car{i}.pos(1) > 0 && mycar.pos(1) - othercars.car{i + 1}.pos(1) < 0
                    car_nr = i;
                    break;
                end
            end
            front_num = car_nr + 1;
            if front_num > othercars.n - 1
                front_num = 1;
            end
            %-----------------------------------------------------------------
            
            %---predict othercar's trajectory and calculate the integral of potential value around the othercar (by yanagihara)-------
            mycar_est = mycar;
            othercars_est = othercars;
            U_r = 0;
            U_f = 0;
            U_rmax = 0;
            U_fmax = 0;
            i_est = 0;
            while mycar_est.pos(2) > -1700
                i_est = i_est + 1;
                othercars_est  = update_othercars_mycar_intelligent(othercars_est, sim, track, mycar_est, car_nr);
                [mycar_est, U_rt, U_ft] = update_control_mycar_est(mycar_est, sim, othercars_est,laneChangePathTranslated,ratioSpeed, car_nr, front_num);
                U_r = U_r + U_rt;
                U_f = U_f + U_ft;
                if U_rt > U_rmax
                   U_rmax = U_rt; 
                end
                if U_ft > U_fmax
                    U_fmax = U_ft;
                end
            end
            U_est_r = U_r / i_est;
            U_est_f = U_f / i_est;
            fprintf(1, 'AVERAGE POTENTIAL (rear)= [%d] \n', U_est_r);
            fprintf(1, 'AVERAGE POTENTIAL (front)= [%d] \n', U_est_f);
            fprintf(1, 'MAXIMUM POTENTIAL INTEGRAL (rear)= [%d] \n', U_rmax);
            fprintf(1, 'MAXIMUM POTENTIAL INTEGRAL (front)= [%d] \n', U_fmax);
%             
%             if U_o > 50000000000
%                 FLAG_LANECHANGE = 0;
%             end
            %-----------------------------------------------------------------------------------------------------------------------
            
        case 'z'
            % decelerate othercar for Intelligent Driving Model
            othercars.car{1}.vel(1) = othercars.car{1}.vel(1) - 1000;
        otherwise 
            fprintf(2, 'KEY[%s] UNDEFINED. \n', key_pressed);
    end
    key_pressed = ''; 
    
    % SIMULATE 
    switch sim.mode 
        case 'RUN'
            % UPDATE
            clk_update = clock;
            sim        = update_sim(sim);
            othercars  = respawn_othercars(othercars,track); % added by kumano
            mycar  = respawn_mycar(mycar,track);
%             othercars  = update_othercars_intelligent(othercars, sim, track); %added by yanagihara
%             mycar    = update_mycar(mycar, sim, othercars);
            
            if FLAG_LANECHANGE ==1
              othercars  = update_othercars_mycar_intelligent(othercars, sim, track, mycar, car_nr); %added by yanagihara
              if FLAG_LANECHANGE_FINISHED ==0
                  mycar    = update_control_mycar(mycar, sim, othercars,laneChangePathTranslated,ratioSpeed);
              else
                  mycar    = update_mycar(mycar, sim, othercars);
              end
            else
              othercars  = update_othercars_intelligent(othercars, sim, track); %added by yanagihara
              mycar    = update_mycar(mycar, sim, othercars);
            end
            
            if mycar.pos(1) > track.xmax
                FLAG_LANECHANGE_FINISHED =1;
            end
            
            
            myinfo     = get_trackinfo(track, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            if FLAG_LANECHANGE == 0
                laneChangePathTranslated = update_laneChangePath(mycar,laneChangePath);
            end
            
            % SAVE TRAJ
            %traj = add_traj(traj, mycar, myinfo);
            
            % TERMINATE CONDITIONS            
            if is_insidetrack(myinfo) == 0 && mycar.pos(1) < track.xmax && mycar.pos(1) > track.xmin
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(track, 2, 0, 1, 0)); % mod by kumano
                FLAG_LANECHANGE = 0;
                FLAG_LANECHANGE_FINISHED = 0;
            end
            if is_carcrashed(myinfo)
                fprintf(2, 'COLLISION OCCURRED. \n');
                mycar.vel = [0 0];
                mycar = init_mycar(get_posintrack(track, 2, 0, 1, 0)); % mod by kumano
                FLAG_LANECHANGE = 0;
                FLAG_LANECHANGE_FINISHED = 0;
            end
        case 'PAUSE'
            titlecol = 'c';
        case 'QUIT'
            sim.flag = 0;
            titlecol = 'r';
    end
    
%     %---mycarと各グリッドの相対角度・位置計算 by Yanagihara----------
%     [theta_grid2me,rho_grid2me] = cart2pol(X - mycar.pos(1),Y - mycar.pos(2));
%     %-----------------------------------------------------------------
%     
%     %---mycarの動特性ポテンシャル計算 by Yanagihara-----------
%     [direction_me, speed_me] = cart2pol(mycar.vel(1)*cos(mycar.pos(3)*pi/180),mycar.vel(1)*sin(mycar.pos(3)*pi/180));
%     mu_me = ones(sizeY,sizeX)*direction_me;
%     C = 1/(2*pi*besseli(0,kappa));             % ベッセル関数を含む定数項
%     vonMises = C * exp(kappa*cos(theta_grid2me - mu_me)); % フォンミーゼス分布の項
%     velTerm = alpha*beta*speed_me*exp(-rho_grid2me/(2*sigma))/(2*pi*sigma); % 速度と距離の項
%     U = vonMises.*velTerm;  % 動特性ポテンシャル
%     %-------------------------------------------
   
    % PLOT
    
    clk_plot = clock;
    FILL_LANES           = 1; % 1
    SIMPLECARSHAPE       = 1; % 0(描画処理が重い場合は SIMPLECARSHAPE=1, REALCARSHAPE=0とする)
    REALCARSHAPE         = 0; % 1 
    PLOT_FUTURE_CARPOSES = 1; % 1
    PLOT_CAR_PATHS       = 1; % 1
    PLOT_RFS             = 1; % 1
    strtemp = ['[%.1fSEC][UPDATE:%.1fMS+PLOT:%.1fMS] ' ...
        '[VEL: %.1fKM/H %.1fDEG/S] \n' ...
        '[%dSEG-%dLANE] / [LANE-DEV DIST:%.1fMM DEG:%.1fDEG] \n' ...
        '[LEFT:%.2fM-CENTER:%.2fM-RIGHT:%.2fM]\n' ...
        '[#SAVE: %d]'];
    titlestr = sprintf(strtemp, sim.sec, ms_update, ms_plot ...
        , mycar.vel(1)/10000*36, mycar.vel(2) ...
        , myinfo.seg_idx, myinfo.lane_idx, myinfo.lane_dev, myinfo.deg ...
        , myinfo.left_fb_dists(1)/1000, myinfo.center_fb_dists(1)/1000 ...
        , myinfo.right_fb_dists(1)/1000 ...
        , traj.data.n);
    titlefontsize = get_fontsize();
    axisinfo = plot_track(track, FILL_LANES);
    plot_axisinfo(axisinfo);
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %----
    plot_laneChangePath(laneChangePathTranslated,FLAG_LANECHANGE); % add by kumano
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    
    
    %---ポテンシャル場描画 by Yanagihara----------
%     itvl= 0:2000:50000;
%     if flagPlot
%        flagPlot = false;
%        [~,h.potential] = contour(X,Y,U,'LevelList',itvl);
%     else
%        h.potential.XData = X;
%        h.potential.YData = Y;
%        h.potential.ZData = U;
%     end
%    
%     caxis([0,50000])
%     colorbar
    
%     figure
%     surf(X,Y,U);
    %--------------------------------------------
    
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    

end
fprintf(2, 'SIMULATION TERMINATED \n');

%%




