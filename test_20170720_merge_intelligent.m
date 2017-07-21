ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;

%========== Simulator Settings ================================ 
% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'TEST_STRAIGHT';
NR_LANE    = 3;
LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.02); % dt = 0.02 [sec]
othercars  = init_othercars_vel(5000);
%nr_cars    = randi([++1 4]);
nr_cars    = 20;  % 11
othercars  = test_addcars_multilane(othercars, track, nr_cars, NR_LANE);
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

% LAMP LANE SETTING
track.lamp_nr_seg = 3;

% MERGING SETTING
selfishness = 0.7; % selfishness of othercar
acc = 500; % acceleration (if othercar is selfish)

%============================================================

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
        
        case 'r'
            % restart simulation (add by yanagihara)
            % INITIALIZE ENVIRONMENT
            TRACK_TYPE = 'TEST_STRAIGHT';
            NR_LANE    = 3;
            LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
            track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
            sim        = init_sim(0.02); % dt = 0.02 [sec]
            othercars  = init_othercars_vel(20000);
            othercars  = test_addcars_multilane(othercars, track, nr_cars, NR_LANE);
            mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0));
            FLAG_LANECHANGE = 0;
            track.lamp_nr_seg = 5;
        case 'w'
            % SAVE TO FILE
            savename = sprintf('data/raw_trajs/traj_%s.mat' ...
                , datestr(datenum(clock),'yyyymmdd_HHMM'));
            save(savename, 'traj');
            fprintf(2, 'TRAJ SAVE TO [%s]. \n', savename);
            sim.mode = 'QUIT';
        
        case 'l'
            % LANE CHANGING (add by kumano)
            FLAG_LANECHANGE = 1;
            FLAG_LANECHANGE_JUDGE = 0; % (0:not yet, 1:go to judge, 2:judge refused, 3:judge preparing, 4:judge cleared) 
            FLAG_LANECHANGE_STOP = 0;
            
            if rand > selfishness
                FLAG_REARCAR_YUZURU = 1;
                fprintf(1, 'REARCAR:YUZURU \n');
            else
                FLAG_REARCAR_YUZURU = 0;
                fprintf(1, 'REARCAR:YUZURANAI \n');
            end
            
            %---get the number of the nearest behind my car (by
            %yanagihara)
            car_nr = 1;
            for i = 1:track.nr_seg
                if mycar.pos(1) - othercars.car{i}.pos(1) > 0 && mycar.pos(1) - othercars.car{i}.pos(1) < mycar.pos(1) - othercars.car{car_nr}.pos(1)
                    car_nr = i;
                end
            end
            
            front_nr = car_nr + 1;
            if front_nr > track.nr_seg
                front_nr = 1;
            end
            %-----------------------------------------------------------------
        case 'z'
            % decelerate othercar for Intelligent Driving Model
            othercars.car{1}.vel(1) = othercars.car{1}.vel(1) - 1000;
            FLAG_INTELLIGENT_DRIVING = 1;
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
            
            if FLAG_LANECHANGE ==1 % added by yanagihara
                
              if (mycar.pos(3) < -7 && FLAG_LANECHANGE_JUDGE == 0) || (FLAG_LANECHANGE_JUDGE == 3 && othercars.car{front_nr}.pos(1) - mycar.pos(1) > 7000)
                  FLAG_LANECHANGE_JUDGE = 1;
              end
              
              if FLAG_LANECHANGE_JUDGE == 1
                  mycar_est = mycar;
                  othercars_est = othercars;
                  FLAG_LANECHANGE_JUDGE = 4;
                  FLAG_LANECHANGE_STOP = 0;
                  while mycar_est.pos(2) > 50
                      %TTCを計算
                      mycar_est = update_control_mycar_est(mycar_est, sim, othercars_est,laneChangePathTranslated,ratioSpeed);
                      othercars_est = update_othercar_est(othercars_est, sim, car_nr);
                      
                      if mycar_est.pos(2) < 2000 && -5000 < mycar_est.pos(1) - othercars_est.car{car_nr}.pos(1) && mycar_est.pos(1) - othercars_est.car{car_nr}.pos(1) < 5000
                          FLAG_LANECHANGE_JUDGE = 2;
                          FLAG_LANECHANGE_STOP =1;
                          break;
                      end
                  end
                  
                  fprintf(1, 'DISTANCE = [%d] \n', mycar_est.pos(1) - othercars_est.car{car_nr}.pos(1));
              end
              
              if FLAG_LANECHANGE_JUDGE == 2 && mycar.pos(1) - othercars.car{car_nr}.pos(1) < 0
                  FLAG_LANECHANGE_JUDGE = 3;
                  front_nr = car_nr;
                  car_nr = car_nr - 1;
                  if car_nr == 0
                      car_nr = car_nr + track.nr_seg;
                  end
                  
                  ransu = rand;
                  fprintf(1, 'RANSU = [%d] \n', ransu);
                  if ransu > selfishness
                      FLAG_REARCAR_YUZURU = 1;
                      fprintf(1, 'REARCAR:YUZURU \n');
                  else
                      FLAG_REARCAR_YUZURU = 0;
                      fprintf(1, 'REARCAR:YUZURANAI \n');
                  end
                  
              elseif FLAG_LANECHANGE_STOP == 0
                  mycar    = update_control_mycar(mycar, sim, othercars,laneChangePathTranslated,ratioSpeed);
              end
              
              if FLAG_REARCAR_YUZURU == 1
                  [othercars, dec] = update_othercars_mycar_intelligent_merge(othercars, sim, track, mycar, car_nr); % added by yanagihara
              else
                  othercars  = update_othercars_intelligent_merge(othercars, sim, track); %added by yanagihara
                  othercars.car{car_nr}.vel(1) = othercars.car{car_nr}.vel(1) + acc;
              end

            else
              mycar    = update_mycar(mycar, sim, othercars);
              othercars  = update_othercars_intelligent_merge(othercars, sim, track); %added by yanagihara
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
            if is_insidetrack(myinfo) == 0
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(track, 2, 0, 1, 0)); % mod by kumano
                FLAG_LANECHANGE = 0;
            end
            if is_carcrashed(myinfo)
                fprintf(2, 'COLLISION OCCURRED. \n');
                mycar.vel = [0 0];
                mycar = init_mycar(get_posintrack(track, 2, 0, 1, 0)); % mod by kumano
                FLAG_LANECHANGE = 0;
            end
        case 'PAUSE'
            titlecol = 'c';
        case 'QUIT'
            sim.flag = 0;
            titlecol = 'r';
    end
   
    % PLOT
    
    clk_plot = clock;
    FILL_LANES           = 1; % 1
    SIMPLECARSHAPE       = 1; % 0(描画処理が重い場合は SIMPLECARSHAPE=1, REALCARSHAPE=0とする)
    REALCARSHAPE         = 0; % 1 
    PLOT_FUTURE_CARPOSES = 1; % 1
    PLOT_CAR_PATHS       = 1; % 1
    PLOT_RFS             = 0; % 1
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
    
    plot([0 90000], [5250 5250], 'r-', 'LineWidth', 4);
    plot([90000 100000], [5250 1750], 'r-', 'LineWidth', 4);
    plot([0 10000 * track.lamp_nr_seg], [1750 1750], 'r-', 'LineWidth', 4);
    
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
    %--------------------------------------------
    
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    

end
fprintf(2, 'SIMULATION TERMINATED \n');

%%




