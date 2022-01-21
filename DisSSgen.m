clear variables; close all;
syms t positive;
syms Iinv [3 3] real;
syms theta [3 1] real;
syms m positive;
syms r1 r2 r3 r4 [3 1] real;
syms Ts positive;

%% for x=[r,sita,dr,omega]
T=[cos(theta(2))*cos(theta(3)),-sin(theta(3)),0;
    cos(theta(2))*sin(theta(3)),cos(theta(3)),0;
    -sin(theta(2)),0,1];
A=sym(zeros(12,12));
A(1:3,7:9)=diag([1,1,1]);
A(4:6,10:12)=inv(T); % Rz' !!!!!!!!!! check here

B=sym(zeros(12,13));
B(7:9,1:3)=diag([1,1,1])/m;
B(7:9,4:6)=diag([1,1,1])/m;
B(7:9,7:9)=diag([1,1,1])/m;
B(7:9,10:12)=diag([1,1,1])/m;
B(10:12,1:3)=Iinv*crossCap(r1);
B(10:12,4:6)=Iinv*crossCap(r2);
B(10:12,7:9)=Iinv*crossCap(r3);
B(10:12,10:12)=Iinv*crossCap(r4);
B(9,13)=-1;

faiT=expm(A*t);
Bd=int(faiT*B,t,0,Ts);
Ad=subs(faiT,t,Ts);

matlabFunction(Ad,Bd,'file','DS_gen','vars',{Ts,m,theta,Iinv,r1,r2,r3,r4});
%% for x=[r,sita,dr,omega,G]
% T=[cos(theta(2))*cos(theta(3)),-sin(theta(3)),0;
%     cos(theta(2))*sin(theta(3)),cos(theta(3)),0;
%     -sin(theta(2)),0,1];
T=[cos(theta(3)),-sin(theta(3)),0;
    sin(theta(3)),cos(theta(3)),0;
    0,0,1];
A=sym(zeros(13,13));
A(1:3,7:9)=diag([1,1,1]);
A(4:6,10:12)=inv(T); % Rz' !!!!!!!!!! check here
A(9,13)=-1;

B=sym(zeros(13,12));
B(7:9,1:3)=diag([1,1,1])/m;
B(7:9,4:6)=diag([1,1,1])/m;
B(7:9,7:9)=diag([1,1,1])/m;
B(7:9,10:12)=diag([1,1,1])/m;
B(10:12,1:3)=Iinv*crossCap(r1);
B(10:12,4:6)=Iinv*crossCap(r2);
B(10:12,7:9)=Iinv*crossCap(r3);
B(10:12,10:12)=Iinv*crossCap(r4);

faiT=expm(A*t);
Bd=int(faiT*B,t,0,Ts);
Ad=subs(faiT,t,Ts);

matlabFunction(Ad,Bd,'file','DS_gen','vars',{Ts,m,theta,Iinv,r1,r2,r3,r4});
matlabFunction(A,B,'file','SS_gen','vars',{m,theta,Iinv,r1,r2,r3,r4});
%matlabFunction(Ad,Bd,'file','DS_gen');

%% subfunction
function M=crossCap(v)
M=[0,-v(3),v(2);
    v(3),0,-v(1);
    -v(2),v(1),0];
end




