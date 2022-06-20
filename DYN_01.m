%% Authors:
% Made by Tran Duc Thien
% Date: 2021/08/31:
% Lagrange approach
%% Total of kinetic energies:
K=1/2*(m1*v01c'*v01c+I1*w1'*w1)+1/2*(m2*v02c'*v02c+I2*w2'*w2);
%% Total of potential energies:
U=-m1*[0 g 0]*P01c-m2*[0 g 0]*P02c;
%% Lagrange function:
L=simplify(K-U);
dLdde1=diff(L,de1);
dLdde2=diff(L,de2);
% dLdde3=diff(L,de3);
dLde1=diff(L,e1);
dLde2=diff(L,e2);
% dLde3=diff(L,e3);
%% Differential
t1=simplify(diff(dLdde1,e1)*de1+diff(dLdde1,e2)*de2+diff(dLdde1,de1)*dde1+diff(dLdde1,de2)*dde2-dLde1);
t2=simplify(diff(dLdde2,e1)*de1+diff(dLdde2,e2)*de2+diff(dLdde2,de1)*dde1+diff(dLdde2,de2)*dde2-dLde2);
% t3=simplify(diff(dLdde3,e1)*de1+diff(dLdde3,e2)*de2+diff(dLdde3,e3)*de3+diff(dLdde3,de1)*dde1+diff(dLdde3,de2)*dde2+diff(dLdde3,de3)*dde3-dLde3);
u = [t1;t2]
%% Compute Inertia matrix:
M=simplify([diff(t1,dde1),diff(t1,dde2);
            diff(t2,dde1),diff(t2,dde2)])
           
%% Compute gravity vector:
G=simplify([diff(U,e1);
            diff(U,e2)])
%% Compute Coriolis matrix
% dM=simplify([diff(M(1,1),e1)*de1+diff(M(1,1),e2)*de2+diff(M(1,1),e3)*de3,diff(M(1,2),e1)*de1+diff(M(1,2),e2)*de2+diff(M(1,2),e3)*de3,diff(M(1,3),e1)*de1+diff(M(1,3),e2)*de2+diff(M(1,3),e3)*de3;...
%              diff(M(2,1),e1)*de1+diff(M(2,1),e2)*de2+diff(M(2,1),e3)*de3,diff(M(2,2),e1)*de1+diff(M(2,2),e2)*de2+diff(M(2,2),e3)*de3,diff(M(2,3),e1)*de1+diff(M(2,3),e2)*de2+diff(M(2,3),e3)*de3;...
%              diff(M(3,1),e1)*de1+diff(M(3,1),e2)*de2+diff(M(3,1),e3)*de3,diff(M(3,2),e1)*de1+diff(M(3,2),e2)*de2+diff(M(3,2),e3)*de3,diff(M(3,3),e1)*de1+diff(M(3,3),e2)*de2+diff(M(3,3),e3)*de3]);
% dMde1=Comp_dMdei(M,e1);
% dMde2=Comp_dMdei(M,e2);
% dMde3=Comp_dMdei(M,e3);
% de=[de1,de2,de3]';
% dde=[dde1,dde2,dde3]';
% t=[t1,t2,t3]';
% C=simplify(dM-1/2*([de'*dMde1;de'*dMde2;de'*dMde3]));
%% check
% simplify(t-M*dde-C*de-G);