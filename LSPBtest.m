% t=12.5105;
% t0=12.5;
% t1=15;

% 
% txx=[0:0.01:2.5];
% for i=1:251
%     thetaaa(i,:) = LSPBtest(txx(i),0,2.5,q0,q1,v0,v1);
% end
% q0=[0 0 0 0 -1.5708 0]';
% q1=[0 -1 0 0 0 0]';
% v0=[0 0 0 0 0 0 ]';
% v1 = [0 0 0 0 0 0]';

function thetas = LSPBtest(t,t0,t1,q0,q1,v0,v1)
tb=(t1-t0)*0.15;
LSPB_matrix = [
1 t0 t0^2 0 0 0 0 0;
0 1 2*t0 0 0 0 0 0;
1 t0+tb (t0+tb)^2 -1 -(t0+tb) 0 0 0;
0 1 2*(t0+tb) 0 -1 0 0 0;
0 0 0 -1 -(t1-tb) 1 t1-tb (t1-tb)^2;
0 0 0 0 -1 0 1 2*(t1-tb);
0 0 0 0 0 1 t1 t1^2;
0 0 0 0 0 0 1 2*t1 ;
];
Q = [ q0'; v0'; zeros(1,6); zeros(1,6); zeros(1,6); zeros(1,6); q1'; v1'];
%Q
%LSPB_matrix
abc = LSPB_matrix \ Q;
b0=abc(1,:);b1=abc(2,:);
b2=abc(3,:);
a0=abc(4,:);a1=abc(5,:);c0=abc(6,:);c1=abc(7,:);c2=abc(8,:);
thetas = zeros(6,1);
for i=1:6
   if t>=t0 && t<t0+tb
       thetas(i)=b0(i)+b1(i)*t+b2(i)*t^2;
   elseif t>=t0+tb && t<= t1-tb
       thetas(i) = a0(i)+a1(i)*t;
   else
       thetas(i)=c0(i)+c1(i)*t+c2(i)*t^2;
   end
end
2*b2*t+b1




