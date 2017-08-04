function plot_winker(mycar)
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end
if first_flag
    first_flag = false;
    %h.winker = viscircles([mycar.pos(1) + 2450, mycar.pos(2) - 1250], 300, 'color', 'y'); %ウインカーの描画
    h.winker = plot(mycar.pos(1) + 2450, mycar.pos(2) - 1250, 'yo');
else
    
    h.winker.XData = mycar.pos(1) + 2450;
    h.winker.YData = mycar.pos(2) - 1250;
    
end