function laneChangePathTranslated = update_laneChangePath(mycar,laneChangePath)

nData = size(laneChangePath,1);
xTrans = ones(nData,1)*(mycar.pos(1));
yTrans = ones(nData,1)*(mycar.pos(2));
trans  = [xTrans,yTrans];

laneChangePathTranslated = laneChangePath + trans;
end
