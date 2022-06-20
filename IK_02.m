clc;
clear all;
close all;
%% THONG SO CUA ROBOT
L1=1; L2=1; L3=1;
g=0;
%% VI TRI CHO TRUOC CUA DIEM EE ROBOT
% the1_d = 1; the2_d = 1; the3_d = 1;
% P_EE = FK(L1,L2,L3,the1_d,the2_d,the3_d);
% x = P_EE(1); y = P_EE(2); z = P_EE(3);
x = 0.5; y=0.5; z=1;

%basic_01: a*cos(x) + b*sin(x) = d
a1 = y;
b1 = -x;
d1 = 0;
k = 0;
if (a1*a1 + b1*b1 - d1*d1) >= 0
    the1 = basic_01(a1, b1, d1);
end
for i=1:2
    a3 = 2*L2*L3;
    b3 = 0;
    d3 = (x*cos(the1(i)) + y*sin(the1(i)) - L1)^2 + z^2 - L2^2 - L3^2;
    if (a3*a3 + b3*b3 - d3*d3) >= 0
        the3 = basic_01(a3, b3, d3);
    end
    for j=1:2
        g=g+1;
        k = k+1;
        A = x*cos(the1(i)) + y*sin(the1(i)) - L1;
        B = z;
        s2 = B*(L3*cos(the3(j))+L2) - A*L3*sin(the3(j));
        c2 = A*(L3*cos(the3(j))+L2) + B*L3*sin(the3(j));
        fprintf('bo nghiem thu %d',k);
        the1_i=the1(i)
        the2 = atan2(s2,c2)
        the3_j=the3(j)
        P_EE = FK_1(L1,L2,L3,the1_i,the2,the3_j)
        %% DRAW i CASE
%         the = [the1(i), the2, the3(j)];
%         draw_dr3(the,g);
%         disp('------------')
    end
end


