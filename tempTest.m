clear variables;
close all;
syms x0 dx0 m real;
syms T H g positive;
l=sqrt(H/g);
xT=x0*cosh(T/l)+l*dx0*sinh(T/l);
dxT=x0/l*sinh(T/l)+dx0*cosh(T/l);

eqn1=xT==-x0;
eqn2=dxT==dx0;

S=solve(eqn1,dx0);

L0=simplify(H*S*m);
pretty(L0)