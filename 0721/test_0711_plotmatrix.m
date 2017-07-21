[A,B,C,D,E,F,G,H,I,J,K,L,M,N,O] = textread('0707_testdata_1000.txt','%d %d %d %f %f %f %f %f %f %f %f %f %f %f %f');
X=[J,K,L,M,O];


figure
gplotmatrix(X, [],D, ['c' 'b' 'm' 'g' 'r'], [], [], false);