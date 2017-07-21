ctlPt = [0 0; 1 0; 2 -1; 3 -1];
P = bezierCurve(ctlPt);
plot(P(:,1),P(:,2))
