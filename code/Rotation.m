syms theta phi psi real
syms f a b c M1 M2 M3 Vz Vx Vy real
syms Ixx Iyy Izz Ixy Ixz Iyz real
Rz=[cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0;0 0 1];
Ry=[cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
Rx=[1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
F=[0;0;f];
Zb=Rx*Ry*Rz*[Vx;Vy;Vz];
Res=inv(Rz)*inv(Ry)*inv(Rx)*F;

disp(Res);
PQR=(diff(Rx*Ry*Rz,'phi')*a+diff(Rx*Ry*Rz,'theta')*b+diff(Rx*Ry*Rz,'psi')*c)*inv(Rz)*inv(Ry)*inv(Rx);
Euler=[a;b;c];
disp(simplify(PQR));
M=[M1;M2;M3];
%I=[Ixx Ixy Ixz;Ixy Iyy Iyz;Ixz Iyz Izz];
%Assume symmetric
I=[Ixx 0 0;0 Iyy 0;0 0 Izz];
T_PQR=[1 0 -sin(theta);0 cos(phi) sin(phi)*cos(theta);0 -sin(phi) cos(phi)*cos(theta)];
EulerAcceleration=inv(I)*M;
disp(EulerAcceleration);