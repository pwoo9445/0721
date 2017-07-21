A = rand(1,2);
B = rand(10,2);
dist=bsxfun(@hypot,B(:,1)-A(1),B(:,2)-A(2));
out = B(dist==min(dist),:);
[M,I]=min(dist);


ctlPt = [0 0; 1 0; 2 -1; 3 -1];
P = bezierCurve(ctlPt);
plot(P(:,1),P(:,2))

ndata =size(P,1);
theta = zeros(ndata,1);
for i = 1:ndata
   if i~=ndata
     vx= P(i+1,1)-P(i,1); 
     vy= P(i+1,2)-P(i,2); 
   else
     vx= P(i,1)-P(i-1,1); 
     vy= P(i,2)-P(i-1,2);   
   end
   theta(i) = atan(vy/vx)*180/pi;
end
