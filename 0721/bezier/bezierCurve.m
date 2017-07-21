function [P, lenghtP] = bezierCurve(ctlPt)
%   BezierCurve function
%
dt =0.005;

n = size(ctlPt,1); % n: number of contrl points
n1=n-1;            % degree of curve
p = ctlPt;

for i=0:1:n1
    sigma(i+1)=factorial(n1)/(factorial(i)*factorial(n1-i));  % for calculating (x!/(y!(x-y)!)) values 
end
l=[];
UB=[];

for u=0:dt:1
    for d=1:n
        UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
    end
    l=cat(1,l,UB);  %catenation 
end

P=l*p;

%---Calc Length--------
nPt = size(P,1);

lenghtP = 0;
for i=1:nPt-1
    v1 = [P(i,1),P(i,2)];
    v2 = [P(i+1,1),P(i+1,2)];
    lenghtP = lenghtP + norm(v2-v1);
end

%----------------------
end
