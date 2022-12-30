clear variables;
close all;
phi=linspace(0,1,1000);
W=0.1;
s=1;
cphi=zeros(length(phi),1);
for i=1:1:length(phi)
    cphi(i)=s*(erf(12*phi(i)/W-6)+erf(12*(1-phi(i))/W-6)-1);
end

figure();
plot(phi,cphi);

zcp=linspace(-1,1,1000);
Cz=zeros(length(zcp),1);
for i=1:1:length(zcp)
    Cz(i)=CzFun(zcp(i));
end
figure();
plot(zcp,Cz);

function y=erf(x)
    y=1/(1+exp(-x));
end

function y=CzFun(x)
kp=100;
kn=20;
if x>=0
    y=exp(-kp*x^2);
else
    y=exp(-kn*x^2);
end
end