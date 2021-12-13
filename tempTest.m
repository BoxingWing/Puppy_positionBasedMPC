clear variables;
close all;
W=diag([1,2,3]);
R=Rz(0.1);
Wnew=R'*W*R





% subfunction
function R=Rz(theta)
R=[cos(theta),-sin(theta),0;
      sin(theta),cos(theta),0;
      0,0,1];
end