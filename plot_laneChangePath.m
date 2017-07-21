function ems = plot_laneChangePath(laneChangePath, FLAG_LANECHANGE)
% PLOT LANE CHANGE PATH
persistent fist_flag h
if isempty(fist_flag)
    fist_flag = true;
end

iclk = clock;
if FLAG_LANECHANGE==1
    if fist_flag
       fist_flag = false;
        h.lane_ch = plot(laneChangePath(:, 1), laneChangePath(:, 2), ':' ...
               , 'Color', [0 1 1], 'LineWidth', 1);
    end 
else
   fist_flag = true;
   h.lane_ch.XData=[];
   h.lane_ch.YData=[];
end
%}

ems = etime(clock, iclk)*1000;