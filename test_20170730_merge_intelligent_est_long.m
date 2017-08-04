ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;
addpath(genpath('./init'))
addpath(genpath('./plot'))
addpath(genpath('./update'))
%========== Simulator Settings ================================
% MERGING SETTING
selfishmode = 1; % 0:random 1:manual(list)
i_list = 1;
selfishlist = {0, 1, 0, 0, 0, 1, 0, 0, 1}; % 0:yuzuranai 1:yuzuru
selfishness = 0.7; % selfishness of othercar
acc_rear = 100; % acceleration (if othercar is selfish)
gap_front_nr = 2;
gap_rear_nr = 1;
acc_mycar = 300; %acceleration of mycar before lanechanging (according to the selected gap)
dec_mycar = 300; %deceleration of mycar before lanechanging (according to the selected gap)


% JUDGEMENT SETTING
% FLAG_JUDGEMENT_TYPE = 1; % 0:TTC, 1:intention estimation by acceleration

% INITIAL VALUE OF OTHERCAR
othercar_vel_ini = 5000;

% PARAMETER OF INTELLIGENT DRIVING MODEL--------------------
idm.v0 = 10000; % desired velocity
idm.T = 1.0; % Safe time headway
idm.a = 4000; % maximum acceleration
idm.b = 6000; %desired deceleration
idm.delta = 4; %acceleration exponent
idm.s0 = 2500; % minimum distance
idm.l = 2500; % vehicle length
%============================================================


% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'TEST_STRAIGHT';
NR_LANE    = 2;
LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.02); % dt = 0.02 [sec]
othercars  = init_othercars();
othercars.npl    = 9;  % number of cars per lane
othercars  = test_addcars_multilane(othercars, track, othercars.npl, NR_LANE, othercar_vel_ini);
mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0));

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL DEMO';
axespos   = [0.03, 0.02, 0.95, 0.84];
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;

% LANE CHANGE SETTING
FLAG_LANECHANGE = 0;
dx = 15000/3; % x-cood.interval of control points for bezier curve
ctlPt = [0 0; dx 0; 2*dx -LANE_WIDTH; 3*dx -LANE_WIDTH];
[laneChangePath, lengthP] = bezierCurve(ctlPt);
ratioSpeed = lengthP/15000*1.1;

% LAMP LANE SETTING
track.lamp_nr_seg = 3;

% GUIGUI LEVEL SETTING
%gui_ego = 1.0; % time length of lanechanging
gui_ego = 1000;
gui_rear = {3000, 3000, 3000, 4500, 3000, 3000, 3000, 3000, 3000}; % acceptable distance of rearcar


%--- grid setting by Yanagihara---------------
% x_grid = 0:200:100000;
% y_grid = -3500:200:3500;
% [X,Y] = meshgrid(x_grid,y_grid);
% [theta,rho] = cart2pol(X,Y);
% sizeX = length(x_grid);
% sizeY = length(y_grid);
%---------------------------------


% flagPlot = true;

% RUN
% INITIALIZE SAVER
traj = init_traj(track, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed
        case ''
        case {'leftarrow', 'semicolon'}
            gap_front_nr = gap_front_nr - 1;
            if gap_front_nr == 0
                gap_front_nr = othercars.npl;
            end
            
            gap_rear_nr = gap_rear_nr - 1;
            if gap_rear_nr == 0
                gap_rear_nr = othercars.npl;
            end
        case {'rightarrow', 'quote'}
            gap_front_nr = gap_front_nr + 1;
            if gap_front_nr > othercars.npl
                gap_front_nr = 1;
            end
            
            gap_rear_nr = gap_rear_nr + 1;
            if gap_rear_nr > othercars.npl
                gap_rear_nr = 1;
            end
        case {'uparrow', 'leftbracket'}
            othercars.car{othercars.npl}.vel(1) = othercars.car{othercars.npl}.vel(1)+2000; % 10000 mm/s = 36 km/h
        case {'downarrow', 'slash'}
            othercars.car{othercars.npl}.vel(1) = othercars.car{othercars.npl}.vel(1)-2000; % 10000 mm/s = 36 km/h
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
            NR_LANE    = 2;
            LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
            track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
            sim        = init_sim(0.02); % dt = 0.02 [sec]
            othercars  = init_othercars();
            othercars.npl    = 9;  % number of cars per lane
            othercars  = test_addcars_multilane(othercars, track, othercars.npl, NR_LANE, othercar_vel_ini);
            mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0));
            FLAG_LANECHANGE = 0;
            track.lamp_nr_seg = 3;
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
            FLAG_LANECHANGE_JUDGE = 0; % (0:not yet, 1:go to judge, 2:judge refused, 3:judge preparing, 4:judge cleared, 5:returning)
            FLAG_LANECHANGE_STOP = 0;
            
%             if (selfishmode == 0 && rand > selfishness) || (selfishmode == 1 && selfishlist{i_list} == 1)
%                 FLAG_REARCAR_YUZURU = 1;
%                 fprintf(1, 'REARCAR:YUZURU \n');
%             else
%                 FLAG_REARCAR_YUZURU = 0;
%                 fprintf(1, 'REARCAR:YUZURANAI \n');
%             end
            
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
            othercars  = respawn_othercars(othercars,track);
            
            if FLAG_LANECHANGE ==2 % entered between gap                
                % judge start               
                if (FLAG_LANECHANGE_JUDGE == 0 || FLAG_LANECHANGE_JUDGE == 3) && i_observe == 10
                    FLAG_LANECHANGE_JUDGE = 1;
                    dx = 15000/3; % x-cood.interval of control points for bezier curve
                    ctlPt = [0 0; dx 0; 2*dx -LANE_WIDTH/2 - mycar.pos(2); 3*dx -LANE_WIDTH/2 - mycar.pos(2)];
                    [laneChangePath, lengthP] = bezierCurve(ctlPt);
                    ratioSpeed = lengthP/15000*1.1;
                    laneChangePathTranslated = update_laneChangePath(mycar,laneChangePath);
                    i_observe = 0;
                end
                
                % observe start                
                if FLAG_LANECHANGE_JUDGE == 5 && othercars.car{gap_front_nr}.pos(1) - mycar.pos(1) > 0
                    
                    i_list = i_list + 1;
                    
                    ransu = rand;
                    fprintf(1, 'RANSU = [%d] \n', ransu);
                    if (selfishmode == 0 && ransu > selfishness) || (selfishmode == 1 && selfishlist{i_list})
%                         FLAG_REARCAR_YUZURU = 1;
%                         fprintf(1, 'REARCAR:YUZURU \n');
                    else
%                         FLAG_REARCAR_YUZURU = 0;
%                         fprintf(1, 'REARCAR:YUZURANAI \n');
                    end
                    
                    FLAG_LANECHANGE_JUDGE = 3;
                end
                
                % lane changing
                if FLAG_LANECHANGE_JUDGE == 4
                    % update mycar speed
                    mycar = update_control_mycar_intelligent(mycar, sim, othercars,laneChangePathTranslated,ratioSpeed, track, gap_front_nr, idm);
                end
                
                if FLAG_LANECHANGE_JUDGE == 0 || FLAG_LANECHANGE_JUDGE == 3
                    A1 = othercars.car{gap_rear_nr}.vel(1)/idm.v0;
                    
                    % calculate acceleration by IDM (follows FRONTCAR)                    
                    A3 = othercars.car{gap_front_nr}.pos(1) - othercars.car{gap_rear_nr}.pos(1) - idm.l;
                    A2 = (idm.s0 + othercars.car{gap_rear_nr}.vel(1)*idm.T + othercars.car{gap_rear_nr}.vel(1) * (othercars.car{gap_rear_nr}.vel(1) - othercars.car{gap_front_nr}.vel(1))/2/sqrt(idm.a*idm.b))/A3;
                    acc_frontcar = idm.a*(1 - A1^idm.delta - A2^2);
                    if othercars.car{gap_rear_nr}.vel(1) + acc_frontcar*sim.T < 0
                        acc_frontcar = othercars.car{gap_rear_nr}.vel(1);
                    end
                    fprintf(1, 'estimated acceleration(follows frontcar) = [%.4d] A3 = [%4d] A2 = [%4d] \n', acc_frontcar, A3, A2);
                    
                    % calculate acceleration by IDM (follows EGOCAR)                    
                    A3 = mycar.pos(1) - othercars.car{gap_rear_nr}.pos(1) - idm.l;
                    A2 = (idm.s0 + othercars.car{gap_rear_nr}.vel(1)*idm.T + othercars.car{gap_rear_nr}.vel(1) * (othercars.car{gap_rear_nr}.vel(1) - mycar.vel(1))/2/sqrt(idm.a*idm.b))/A3;
                    acc_mycar = idm.a*(1 - A1^idm.delta - A2^2);
                    if othercars.car{gap_rear_nr}.vel(1) + acc_mycar*sim.T < 0
                        acc_mycar = othercars.car{gap_rear_nr}.vel(1);
                    end
                    fprintf(1, 'estimated acceleration(follows mycar) = [%.4d] A3 = [%4d] A2 = [%4d] \n', acc_mycar, A3, A2);
                    
                    vel_rear_st = othercars.car{gap_rear_nr}.vel(1);
                    
                    i_observe = i_observe + 1;
                end
                
                % update othercars speed
                if selfishlist{gap_rear_nr} == 1
                    [othercars, dec] = update_othercars_mycar_intelligent_merge(othercars, sim, track, mycar, gap_rear_nr, idm);
                    fprintf(2, 'deceleration(rearcar) = [%.4d]\n', dec);
                else
                    othercars  = update_othercars_intelligent_merge(othercars, sim, track, idm);
                    %othercars.car{gap_rear_nr}.vel(1) = othercars.car{gap_rear_nr}.vel(1) + acc_rear;
                end
                
                if FLAG_LANECHANGE_JUDGE == 2
                    if mycar.pos(2) > gui_ego
                        if sqrt((mycar.pos(1) - othercars.car{gap_rear_nr}.pos(1))^2 + (mycar.pos(2) - othercars.car{gap_rear_nr}.pos(2))^2) < gui_rear{gap_rear_nr}
                            FLAG_LANECHANGE_JUDGE = 4;
                            selfishlist{gap_rear_nr} = 1;
                        end
                        i_nego = i_nego + sim.T;
                        mycar = update_control_mycar_intelligent(mycar, sim, othercars,laneChangePathTranslated,ratioSpeed, track, gap_front_nr, idm);
                        if mycar.vel(1) < 3000
                            mycar.vel(1) = 3000;
                        end
                        fprintf(1, 'time_nego = [%4d] distance = [%4d] mycar.pos(2) = [%4d] \n', i_nego, sqrt((mycar.pos(1) - othercars.car{gap_rear_nr}.pos(1))^2 + (mycar.pos(2) - othercars.car{gap_rear_nr}.pos(2))^2), mycar.pos(2));
                    else
                        FLAG_LANECHANGE_JUDGE = 5;
                        % make bezier curve when returns current lane
                        dx = 2000; % x-cood.interval of control points for bezier curve
                        dy = LANE_WIDTH/2 - mycar.pos(2);
                        ctlPt = [0 0; dx 0; 2*dx dy; 3*dx dy];
                        [laneChangePath_r, lengthP_r] = bezierCurve(ctlPt);
                        ratioSpeed_r = lengthP_r/15000*1.1;
                        laneChangePathTranslated_r = update_laneChangePath(mycar,laneChangePath_r);
                        %plot_laneChangePath(laneChangePathTranslated,FLAG_LANECHANGE);
                        
                        gap_front_nr = gap_rear_nr;
                        gap_rear_nr = gap_rear_nr - 1;
                        if gap_rear_nr == 0
                            gap_rear_nr = gap_rear_nr + othercars.npl;
                        end
                    end
                end
                    
                
                if FLAG_LANECHANGE_JUDGE == 0 || FLAG_LANECHANGE_JUDGE == 1 || FLAG_LANECHANGE_JUDGE == 3
                    
                    [mycar, dec_me] = update_mycar_intelligent(mycar, sim, othercars, track, gap_front_nr, idm);
                    
                    % observe acceleration for comparison
                    vel_rear_en = othercars.car{gap_rear_nr}.vel(1);
                    if abs((vel_rear_en - vel_rear_st)/sim.T - acc_frontcar) < abs((vel_rear_en - vel_rear_st)/sim.T - acc_mycar)
                        fprintf(2, 'following FRONTCAR\n');
                    else
                        fprintf(2, 'following MYCAR\n');
                    end
                    fprintf(1, 'acceleration(actual) = [%.4d]\n', (vel_rear_en - vel_rear_st)/sim.T);
                    fprintf(1, 'mycar.vel(1) = [%.4d]\n', mycar.vel(1));
                    fprintf(2, '[%d]\n', i_observe);
                end
                
                % return current lane
                if FLAG_LANECHANGE_JUDGE == 5
                    
                    % update mycar speed
                    mycar.vel(1) = mycar.vel(1) - 1000;
                    mycar = update_control_mycar(mycar, sim, othercars,laneChangePathTranslated_r,ratioSpeed_r);
                    if mycar.vel(1) < 3000
                        mycar.vel(1) = 3000;
                    end
                    
                end
                
                
                
                % decide whether change lane or not (1st step)                
                if FLAG_LANECHANGE_JUDGE == 1
                    FLAG_LANECHANGE_JUDGE = 4;
                    
                    vel_rear_en = othercars.car{gap_rear_nr}.vel(1);
                    if abs((vel_rear_en - vel_rear_st)/sim.T - acc_frontcar) < abs((vel_rear_en - vel_rear_st)/sim.T - acc_mycar)
                        FLAG_LANECHANGE_JUDGE = 2;
                        i_nego = 0;
                    end
                    fprintf(1, 'acceleration(actual) = [%.4d]\n', (vel_rear_en - vel_rear_st)/sim.T);
                end
                
            elseif FLAG_LANECHANGE ==1 % imput lane changing
                
                % turn on blinker when egocar enters the gap                
                if mycar.pos(1) < othercars.car{gap_front_nr}.pos(1) && mycar.pos(1) - 1000 > othercars.car{gap_rear_nr}.pos(1) + 1000
                    FLAG_LANECHANGE = 2;
                    % turn Signal
                    mycar.turnSignal = 'right';
                    i_observe = 0;
                end
                
                % decelerate if egocar exists in front of the gap
                if mycar.pos(1) > othercars.car{gap_front_nr}.pos(1) - 1000
                    mycar.vel(1) = mycar.vel(1) - dec_mycar;
                    if mycar.vel(1) < 0
                        mycar.vel(1) = 0;
                    end
                else
                    [mycar, dec_me] = update_mycar_intelligent(mycar, sim, othercars, track, gap_front_nr, idm);
                    mycar.vel(1) = mycar.vel(1) + acc_mycar;
                end
                
                othercars  = update_othercars_intelligent_merge(othercars, sim, track, idm); %added by yanagihara
                laneChangePathTranslated = update_laneChangePath(mycar,laneChangePath);
                
            else % before lanechanging is input
                mycar    = update_mycar(mycar, sim, othercars);
                othercars  = update_othercars_intelligent_merge(othercars, sim, track, idm); %added by yanagihara
            end
            
            myinfo     = get_trackinfo(track, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
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
    SIMPLECARSHAPE       = 1; % 0
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
    axisinfo = plot_track_merge(track, FILL_LANES);
    plot_axisinfo(axisinfo);
    %plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    plot_othercars_gap(othercars, SIMPLECARSHAPE, REALCARSHAPE, gap_front_nr, gap_rear_nr, selfishlist);
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, PLOT_RFS);
    plot_turnSignal(mycar,sim);
    %plot_traj(traj);
    %----
    if FLAG_LANECHANGE == 2
        plot_laneChangePath(laneChangePathTranslated,FLAG_LANECHANGE); % add by kumano
    end
    %----
    % plot_title(titlestr, titlecol, titlefontsize);
    
    %---plot potential field by Yanagihara----------
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




