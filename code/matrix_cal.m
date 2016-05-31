syms KL1 KL2 KL3 KL4 KD1 KD2 KD3 KD4
syms gT1 gT2 gT3 gM1 gM2 gM3
syms u1 u2 u3 u4 real

%     FT = (kL(1)*u1 + kL(2)*u2 + kL(3)*u2 + kL(4)*u4);
%     M1 = (kL(4)*u4 - kL(2)*u2); 
%     M2 = (-kL(1)*u1 + kL(3)*u3); 
%     M3 = (kD(1)*u1 + kD(3)*u3) - (kD(2)*u2 + kD(4)*u4);

B = [KL1 KL2 KL3 KL4; 0 -KL2 0 KL4; -KL1 0 KL3 0; KD1 -KD2 KD3 -KD4];
U = [u1 u2 u3 u4]';
C = B*U;
D = [[gT1; gT2; gT3], zeros(3,3); zeros(3,1) diag([gM1; gM2; gM3])];
E = D*C;